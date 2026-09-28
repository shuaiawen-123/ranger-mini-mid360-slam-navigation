#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal
import subprocess
import sys
import time
from typing import List, Optional


class FullNavigationManager:
    """
    全导航系统外层管理器。

    启动：
        roslaunch ranger_navigation full_navigation.launch

    第一次收到 Ctrl+C：
        1. 取消 move_base 当前目标
        2. 单独关闭 move_base，阻止其继续发布速度
        3. 保持 ranger_base_node 在线
        4. 以固定频率连续发布零速度
        5. 最后关闭整个 roslaunch

    第二次收到 Ctrl+C：
        强制结束整个 roslaunch 进程组
    """

    def __init__(self) -> None:
        self.launch_process: Optional[subprocess.Popen] = None
        self.shutdown_requested = False
        self.shutdown_running = False
        self.force_shutdown_requested = False

        self.zero_publish_rate = 20.0
        self.zero_publish_duration = 1.5

        self.zero_twist = (
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, "
            "angular: {x: 0.0, y: 0.0, z: 0.0}}"
        )

    @staticmethod
    def log(message: str) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [navigation_manager] {message}", flush=True)

    @staticmethod
    def run_command(
        command: List[str],
        timeout: float = 3.0,
        show_output: bool = False
    ) -> subprocess.CompletedProcess:
        stdout_target = None if show_output else subprocess.PIPE
        stderr_target = None if show_output else subprocess.STDOUT

        return subprocess.run(
            command,
            stdout=stdout_target,
            stderr=stderr_target,
            text=True,
            timeout=timeout,
            check=False
        )

    def ros_master_available(self) -> bool:
        try:
            result = self.run_command(
                ["rosnode", "list"],
                timeout=2.0,
                show_output=False
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, OSError):
            return False

    def ros_node_exists(self, node_name: str) -> bool:
        try:
            result = self.run_command(
                ["rosnode", "list"],
                timeout=2.0,
                show_output=False
            )

            if result.returncode != 0:
                return False

            output = result.stdout or ""
            nodes = [line.strip() for line in output.splitlines()]
            return node_name in nodes

        except (subprocess.TimeoutExpired, OSError):
            return False

    def cancel_navigation_goal(self) -> None:
        if not self.ros_node_exists("/move_base"):
            self.log("未检测到 /move_base，跳过目标取消。")
            return

        self.log("步骤1/4：取消当前 move_base 导航目标。")

        try:
            result = self.run_command(
                [
                    "rostopic",
                    "pub",
                    "-1",
                    "/move_base/cancel",
                    "actionlib_msgs/GoalID",
                    "{}"
                ],
                timeout=3.0,
                show_output=False
            )

            if result.returncode == 0:
                self.log("导航目标取消命令已发送。")
            else:
                self.log(
                    "导航目标取消命令返回非零状态，继续执行安全停车。"
                )

        except subprocess.TimeoutExpired:
            self.log("取消导航目标超时，继续执行安全停车。")
        except OSError as error:
            self.log(f"取消导航目标失败：{error}")

    def stop_move_base(self) -> None:
        """
        先单独关闭 move_base，确保它不再与零速度发布者竞争。
        Ranger 底盘节点此时仍保持运行。
        """
        if not self.ros_node_exists("/move_base"):
            self.log("未检测到 /move_base，无需单独关闭。")
            return

        self.log("步骤2/4：单独关闭 /move_base，停止新的速度指令。")

        try:
            self.run_command(
                ["rosnode", "kill", "/move_base"],
                timeout=4.0,
                show_output=False
            )
        except subprocess.TimeoutExpired:
            self.log("关闭 /move_base 超时，继续发布零速度。")
        except OSError as error:
            self.log(f"关闭 /move_base 失败：{error}")

        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            if not self.ros_node_exists("/move_base"):
                self.log("/move_base 已停止。")
                return
            time.sleep(0.1)

        self.log("/move_base 尚未完全注销，仍继续执行零速度停车。")

    def publish_zero_velocity(self) -> None:
        """
        Ranger 驱动仍在线时，连续发送零速度。
        """
        self.log(
            "步骤3/4：保持底盘驱动在线，"
            f"以 {self.zero_publish_rate:.0f} Hz "
            f"连续发送 {self.zero_publish_duration:.1f} 秒零速度。"
        )

        command = [
            "rostopic",
            "pub",
            "-r",
            str(self.zero_publish_rate),
            "/cmd_vel",
            "geometry_msgs/Twist",
            self.zero_twist
        ]

        zero_process: Optional[subprocess.Popen] = None

        try:
            zero_process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True
            )

            deadline = (
                time.monotonic() + self.zero_publish_duration
            )

            while time.monotonic() < deadline:
                if self.force_shutdown_requested:
                    break

                if zero_process.poll() is not None:
                    self.log(
                        "零速度发布进程提前退出，尝试发送最后一帧。"
                    )
                    break

                time.sleep(0.05)

        except OSError as error:
            self.log(f"启动零速度发布进程失败：{error}")

        finally:
            if zero_process is not None and zero_process.poll() is None:
                try:
                    os.killpg(
                        os.getpgid(zero_process.pid),
                        signal.SIGINT
                    )
                    zero_process.wait(timeout=1.0)
                except (
                    ProcessLookupError,
                    subprocess.TimeoutExpired,
                    OSError
                ):
                    try:
                        os.killpg(
                            os.getpgid(zero_process.pid),
                            signal.SIGKILL
                        )
                    except (ProcessLookupError, OSError):
                        pass

        # 再补发最后一帧零速度
        try:
            self.run_command(
                [
                    "rostopic",
                    "pub",
                    "-1",
                    "/cmd_vel",
                    "geometry_msgs/Twist",
                    self.zero_twist
                ],
                timeout=2.0,
                show_output=False
            )
        except (subprocess.TimeoutExpired, OSError):
            pass

        self.log("零速度安全停车指令发送完成。")

    def stop_full_launch(self) -> None:
        if self.launch_process is None:
            return

        if self.launch_process.poll() is not None:
            return

        self.log("步骤4/4：开始关闭完整导航 roslaunch。")

        try:
            os.killpg(
                os.getpgid(self.launch_process.pid),
                signal.SIGINT
            )
        except (ProcessLookupError, OSError):
            return

        try:
            self.launch_process.wait(timeout=15.0)
            self.log("完整导航系统已正常关闭。")
            return
        except subprocess.TimeoutExpired:
            self.log("正常关闭超过 15 秒，发送 SIGTERM。")

        try:
            os.killpg(
                os.getpgid(self.launch_process.pid),
                signal.SIGTERM
            )
        except (ProcessLookupError, OSError):
            return

        try:
            self.launch_process.wait(timeout=5.0)
            self.log("完整导航系统已通过 SIGTERM 关闭。")
            return
        except subprocess.TimeoutExpired:
            self.log("仍有残留进程，执行最终强制结束。")

        try:
            os.killpg(
                os.getpgid(self.launch_process.pid),
                signal.SIGKILL
            )
        except (ProcessLookupError, OSError):
            pass

    def perform_safe_shutdown(self) -> None:
        if self.shutdown_running:
            return

        self.shutdown_running = True

        self.log("收到退出请求，开始严格顺序安全停车。")

        if self.ros_master_available():
            self.cancel_navigation_goal()
            time.sleep(0.2)

            self.stop_move_base()
            time.sleep(0.2)

            self.publish_zero_velocity()
            time.sleep(0.2)
        else:
            self.log(
                "ROS Master 当前不可用，跳过 ROS 停车命令，"
                "直接关闭启动进程。"
            )

        self.stop_full_launch()

    def signal_handler(self, signum: int, _frame) -> None:
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.log(
                f"收到信号 {signum}。"
                "请等待安全停车完成，不要连续按 Ctrl+C。"
            )
            return

        self.force_shutdown_requested = True
        self.log("再次收到退出信号，执行强制关闭。")

        if (
            self.launch_process is not None
            and self.launch_process.poll() is None
        ):
            try:
                os.killpg(
                    os.getpgid(self.launch_process.pid),
                    signal.SIGKILL
                )
            except (ProcessLookupError, OSError):
                pass

    def start(self, extra_launch_args: List[str]) -> int:
        command = [
            "roslaunch",
            "ranger_navigation",
            "full_navigation.launch"
        ] + extra_launch_args

        self.log("启动完整导航系统：")
        self.log(" ".join(command))

        try:
            # 子 roslaunch 单独建立进程组。
            # 终端 Ctrl+C 只先发给本管理脚本，
            # 不会同时直接杀死 Ranger 驱动。
            self.launch_process = subprocess.Popen(
                command,
                start_new_session=True
            )
        except OSError as error:
            self.log(f"启动 roslaunch 失败：{error}")
            return 1

        while True:
            if self.shutdown_requested:
                self.perform_safe_shutdown()
                break

            return_code = self.launch_process.poll()

            if return_code is not None:
                self.log(
                    f"roslaunch 已自行退出，返回码：{return_code}"
                )
                return return_code

            time.sleep(0.1)

        return_code = self.launch_process.poll()

        if return_code is None:
            return_code = 0

        return return_code


def main() -> int:
    manager = FullNavigationManager()

    signal.signal(signal.SIGINT, manager.signal_handler)
    signal.signal(signal.SIGTERM, manager.signal_handler)

    extra_launch_args = sys.argv[1:]

    return manager.start(extra_launch_args)


if __name__ == "__main__":
    sys.exit(main())
