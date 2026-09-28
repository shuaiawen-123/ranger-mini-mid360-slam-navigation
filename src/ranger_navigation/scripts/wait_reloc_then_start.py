#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import signal
import subprocess
import time

import rospy
import tf2_ros


class NavigationStarter:
    """
    启动流程：

    1. 等待用户在 RViz 发布 /initialpose
    2. 等待 /slam_reloc_check 返回定位成功
    3. 启动 nav_map -> odom 桥接
    4. 等待 nav_map -> base_link TF 建立
    5. 启动 TEB move_base
    """

    def __init__(self):
        self.initial_pose_received = False
        self.children = []

        self.reloc_service = rospy.get_param(
            "~reloc_service",
            "/slam_reloc_check"
        )

        self.bridge_package = rospy.get_param(
            "~bridge_package",
            "ranger_localization_bridge"
        )
        self.bridge_launch = rospy.get_param(
            "~bridge_launch",
            "nav_map_odom_bridge.launch"
        )

        self.navigation_package = rospy.get_param(
            "~navigation_package",
            "ranger_navigation"
        )
        self.navigation_launch = rospy.get_param(
            "~navigation_launch",
            "move_base_teb.launch"
        )

        self.settle_time = float(
            rospy.get_param("~settle_time", 3.0)
        )

        self.auto_start_teb = bool(
            rospy.get_param("~auto_start_teb", True)
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(20.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        # 不依赖具体消息类型，只要收到 /initialpose 即可。
        self.initial_pose_subscriber = rospy.Subscriber(
            "/initialpose",
            rospy.AnyMsg,
            self.initial_pose_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.shutdown_children)

    def initial_pose_callback(self, _message):
        if not self.initial_pose_received:
            rospy.loginfo(
                "检测到 RViz 2D Pose Estimate，开始等待重定位结果。"
            )
        self.initial_pose_received = True

    def localization_is_successful(self):
        """
        使用 rosservice CLI 调用自定义服务，
        避免在脚本中依赖 FASTLIO 自定义 srv Python 类型。
        """
        try:
            result = subprocess.run(
                [
                    "rosservice",
                    "call",
                    self.reloc_service,
                    "code: true"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3.0,
                check=False
            )
        except (subprocess.TimeoutExpired, OSError) as error:
            rospy.logwarn_throttle(
                3.0,
                "检查重定位状态失败：%s",
                str(error)
            )
            return False

        output = result.stdout or ""

        # 兼容 status: true、status: True、status: 1
        matched = re.search(
            r"status\s*:\s*(true|1)\b",
            output,
            flags=re.IGNORECASE
        )

        return matched is not None

    def start_roslaunch(self, package_name, launch_name):
        command = [
            "roslaunch",
            package_name,
            launch_name
        ]

        rospy.loginfo(
            "启动：roslaunch %s %s",
            package_name,
            launch_name
        )

        process = subprocess.Popen(
            command,
            preexec_fn=os.setsid
        )

        self.children.append(process)
        return process

    def wait_for_navigation_tf(self, bridge_process):
        """
        等待：
            nav_map -> odom -> base_link
        真正连通后再启动 move_base。
        """
        stable_count = 0

        while not rospy.is_shutdown():
            if bridge_process.poll() is not None:
                raise RuntimeError(
                    "nav_map_odom_bridge 已异常退出"
                )

            try:
                self.tf_buffer.lookup_transform(
                    "nav_map",
                    "base_link",
                    rospy.Time(0),
                    rospy.Duration(0.5)
                )

                stable_count += 1

                if stable_count >= 5:
                    rospy.loginfo(
                        "nav_map -> base_link TF 已稳定建立。"
                    )
                    return

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException
            ):
                stable_count = 0
                rospy.loginfo_throttle(
                    3.0,
                    "等待 nav_map -> base_link TF..."
                )

            rospy.sleep(0.2)

    def monitor_children(self):
        while not rospy.is_shutdown():
            for process in self.children:
                if process.poll() is not None:
                    rospy.logerr(
                        "子 roslaunch 已退出，返回码：%s",
                        str(process.returncode)
                    )
            rospy.sleep(1.0)

    def shutdown_children(self):
        """
        主 launch 按 Ctrl+C 时，一并关闭脚本启动的两个子 launch。
        """
        for process in reversed(self.children):
            if process.poll() is not None:
                continue

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGINT
                )
            except (ProcessLookupError, OSError):
                pass

        deadline = time.time() + 5.0

        for process in reversed(self.children):
            if process.poll() is not None:
                continue

            remaining = max(0.0, deadline - time.time())

            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGTERM
                    )
                except (ProcessLookupError, OSError):
                    pass

    def run(self):
        rospy.loginfo(
            "系统基础节点已启动，请在 RViz 中执行 2D Pose Estimate。"
        )

        while (
            not rospy.is_shutdown()
            and not self.initial_pose_received
        ):
            rospy.loginfo_throttle(
                5.0,
                "等待 RViz /initialpose..."
            )
            rospy.sleep(0.2)

        if rospy.is_shutdown():
            return

        while (
            not rospy.is_shutdown()
            and not self.localization_is_successful()
        ):
            rospy.loginfo_throttle(
                3.0,
                "已收到初始位姿，等待 FASTLIO2_SAM_LC 重定位成功..."
            )
            rospy.sleep(1.0)

        if rospy.is_shutdown():
            return

        rospy.loginfo(
            "FASTLIO2_SAM_LC 重定位成功，等待 %.1f 秒使结果稳定。",
            self.settle_time
        )
        rospy.sleep(self.settle_time)

        bridge_process = self.start_roslaunch(
            self.bridge_package,
            self.bridge_launch
        )

        self.wait_for_navigation_tf(bridge_process)

        if self.auto_start_teb:
            self.start_roslaunch(
                self.navigation_package,
                self.navigation_launch
            )

            rospy.loginfo(
                "TEB 导航已自动启动，可以使用 RViz 2D Nav Goal。"
            )
        else:
            rospy.loginfo(
                "定位桥接已启动，auto_start_teb=false，未启动 TEB。"
            )

        self.monitor_children()


def main():
    rospy.init_node("wait_reloc_then_start")

    starter = NavigationStarter()

    try:
        starter.run()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as error:
        rospy.logfatal(str(error))
        rospy.signal_shutdown(str(error))


if __name__ == "__main__":
    main()
