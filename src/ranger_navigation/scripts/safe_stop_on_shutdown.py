#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import sys
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist


class SafeStopNode:
    def __init__(self):
        self.stopping = False

        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", "/cmd_vel"
        )
        self.cancel_topic = rospy.get_param(
            "~cancel_topic", "/move_base/cancel"
        )
        self.publish_rate = float(
            rospy.get_param("~publish_rate", 20.0)
        )
        self.publish_duration = float(
            rospy.get_param("~publish_duration", 1.0)
        )

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=10
        )

        self.cancel_pub = rospy.Publisher(
            self.cancel_topic,
            GoalID,
            queue_size=10
        )

    def safe_stop(self):
        if self.stopping:
            return

        self.stopping = True

        rospy.logwarn(
            "收到关闭信号：取消导航目标并连续发布零速度。"
        )

        cancel_msg = GoalID()
        zero_msg = Twist()

        # 先多次取消当前 move_base 目标
        for _ in range(5):
            try:
                self.cancel_pub.publish(cancel_msg)
            except Exception:
                pass
            time.sleep(0.05)

        # 然后连续发送零速度，防止被最后一帧速度覆盖
        count = max(
            1,
            int(self.publish_rate * self.publish_duration)
        )
        interval = 1.0 / max(self.publish_rate, 1.0)

        for _ in range(count):
            try:
                self.cmd_pub.publish(zero_msg)
            except Exception:
                pass
            time.sleep(interval)

        rospy.logwarn("零速度安全停车指令发送完成。")

    def signal_handler(self, signum, _frame):
        self.safe_stop()
        rospy.signal_shutdown(
            "received signal {}".format(signum)
        )


def main():
    # 使用自定义信号处理，确保先发送停车指令，再关闭 ROS 节点
    rospy.init_node(
        "safe_stop_on_shutdown",
        disable_signals=True
    )

    node = SafeStopNode()

    signal.signal(signal.SIGINT, node.signal_handler)
    signal.signal(signal.SIGTERM, node.signal_handler)

    rospy.loginfo(
        "安全停车节点已启动，等待系统关闭信号。"
    )

    try:
        while not rospy.is_shutdown():
            time.sleep(0.2)
    finally:
        node.safe_stop()


if __name__ == "__main__":
    main()
