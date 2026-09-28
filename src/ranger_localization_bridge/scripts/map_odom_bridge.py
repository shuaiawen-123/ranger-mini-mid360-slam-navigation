#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def normalize_angle(angle):
    """将角度限制到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_xy_yaw(transform):
    """从 TransformStamped 中提取二维 x、y、yaw。"""
    t = transform.transform.translation
    q = transform.transform.rotation

    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return t.x, t.y, yaw


class MapOdomBridge:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self.transform_tolerance = float(
            rospy.get_param("~transform_tolerance", 0.20)
        )

        # 0 < alpha <= 1
        # 越小越平滑，但响应越慢；1 表示不滤波。
        self.alpha = float(rospy.get_param("~alpha", 0.15))
        self.alpha = max(0.01, min(1.0, self.alpha))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.initialized = False
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0

        rospy.loginfo(
            "Map-Odom bridge started: %s -> %s",
            self.map_frame,
            self.odom_frame
        )
        rospy.loginfo(
            "Using planar assumption: %s and %s have the same x/y center",
            self.body_frame,
            self.base_frame
        )

    def calculate_transform(self):
        # T_map_body：FASTLIO 给出的雷达在地图中的全局位置
        map_to_body = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.body_frame,
            rospy.Time(0),
            rospy.Duration(0.2)
        )

        # T_odom_base：Ranger 轮式里程计
        odom_to_base = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(0.2)
        )

        x_mb, y_mb, yaw_mb = get_xy_yaw(map_to_body)
        x_ob, y_ob, yaw_ob = get_xy_yaw(odom_to_base)

        # T_map_odom = T_map_base * inverse(T_odom_base)
        # 因为在二维平面内假设 body 与 base_link 重合。
        yaw_mo = normalize_angle(yaw_mb - yaw_ob)

        cos_yaw = math.cos(yaw_mo)
        sin_yaw = math.sin(yaw_mo)

        x_mo = x_mb - cos_yaw * x_ob + sin_yaw * y_ob
        y_mo = y_mb - sin_yaw * x_ob - cos_yaw * y_ob

        return x_mo, y_mo, yaw_mo

    def filter_transform(self, x, y, yaw):
        if not self.initialized:
            self.filtered_x = x
            self.filtered_y = y
            self.filtered_yaw = yaw
            self.initialized = True
            return

        a = self.alpha

        self.filtered_x += a * (x - self.filtered_x)
        self.filtered_y += a * (y - self.filtered_y)

        yaw_error = normalize_angle(yaw - self.filtered_yaw)
        self.filtered_yaw = normalize_angle(
            self.filtered_yaw + a * yaw_error
        )

    def publish_transform(self):
        q = quaternion_from_euler(0.0, 0.0, self.filtered_yaw)

        msg = TransformStamped()
        msg.header.stamp = (
            rospy.Time.now()
            + rospy.Duration(self.transform_tolerance)
        )
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame

        msg.transform.translation.x = self.filtered_x
        msg.transform.translation.y = self.filtered_y
        msg.transform.translation.z = 0.0

        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(msg)

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                x, y, yaw = self.calculate_transform()
                self.filter_transform(x, y, yaw)
                self.publish_transform()

                rospy.loginfo_throttle(
                    5.0,
                    "Publishing map->odom: x=%.3f, y=%.3f, yaw=%.2f deg",
                    self.filtered_x,
                    self.filtered_y,
                    math.degrees(self.filtered_yaw)
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException
            ) as error:
                rospy.logwarn_throttle(
                    3.0,
                    "Waiting for required TF: %s",
                    str(error)
                )

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("map_odom_bridge")

    try:
        bridge = MapOdomBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
