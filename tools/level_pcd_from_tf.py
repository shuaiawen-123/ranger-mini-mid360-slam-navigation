#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time

import numpy as np
import rospy
import tf2_ros
from tf.transformations import (
    euler_from_quaternion,
    quaternion_matrix,
)


def average_quaternions(quaternions):
    """
    使用特征向量方法计算多个四元数的平均值。
    四元数顺序为 [x, y, z, w]。
    """
    reference = quaternions[0]
    matrix = np.zeros((4, 4), dtype=np.float64)

    for quaternion in quaternions:
        q = np.asarray(quaternion, dtype=np.float64)

        # q 与 -q 表示相同旋转，先统一符号。
        if np.dot(q, reference) < 0.0:
            q = -q

        q /= np.linalg.norm(q)
        matrix += np.outer(q, q)

    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    result = eigenvectors[:, np.argmax(eigenvalues)]

    if np.dot(result, reference) < 0.0:
        result = -result

    return result / np.linalg.norm(result)


def collect_transform(parent_frame, child_frame, samples, sample_rate):
    """
    获取 T_parent_child：
    即 child 坐标系在 parent 坐标系中的位姿。
    """
    buffer = tf2_ros.Buffer(rospy.Duration(20.0))
    listener = tf2_ros.TransformListener(buffer)

    rospy.sleep(1.0)

    translations = []
    quaternions = []

    rate = rospy.Rate(sample_rate)

    rospy.loginfo(
        "开始采集 TF：%s -> %s，共 %d 个样本",
        parent_frame,
        child_frame,
        samples,
    )

    while not rospy.is_shutdown() and len(translations) < samples:
        try:
            transform = buffer.lookup_transform(
                parent_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            translations.append([
                translation.x,
                translation.y,
                translation.z,
            ])

            quaternions.append([
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ])

            if len(translations) % 20 == 0:
                rospy.loginfo(
                    "已采集 %d/%d",
                    len(translations),
                    samples,
                )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn("等待 TF：%s", str(error))

        rate.sleep()

    if len(translations) < samples:
        raise RuntimeError("没有采集到足够的 TF 数据")

    translation_average = np.mean(
        np.asarray(translations, dtype=np.float64),
        axis=0,
    )
    quaternion_average = average_quaternions(quaternions)

    # 防止 listener 被提前回收。
    _ = listener

    return translation_average, quaternion_average


def transform_ascii_pcd(input_path, output_path, translation, quaternion):
    """
    输入点位于旧 map 坐标系。

    已知 T_map_local（local 在 map 中的位姿），
    将点转换为 local/nav_map 坐标：

        P_local = inverse(T_map_local) * P_map
    """
    rotation_map_local = quaternion_matrix(quaternion)[:3, :3]

    # 逆变换
    rotation_local_map = rotation_map_local.T

    with open(input_path, "r", encoding="utf-8", errors="ignore") as source:
        lines = source.readlines()

    fields = None
    data_line_index = None

    for index, line in enumerate(lines):
        stripped = line.strip()

        if stripped.startswith("FIELDS "):
            fields = stripped.split()[1:]

        if stripped.lower() == "data ascii":
            data_line_index = index
            break

    if fields is None:
        raise RuntimeError("PCD 文件中没有 FIELDS")

    if data_line_index is None:
        raise RuntimeError("输入必须是 ASCII PCD：DATA ascii")

    for required_field in ("x", "y", "z"):
        if required_field not in fields:
            raise RuntimeError(
                "PCD 中缺少字段：{}".format(required_field)
            )

    x_index = fields.index("x")
    y_index = fields.index("y")
    z_index = fields.index("z")

    point_count = 0

    with open(output_path, "w", encoding="utf-8") as target:
        # 保留原始 PCD 头部。
        for line in lines[:data_line_index + 1]:
            target.write(line)

        for line in lines[data_line_index + 1:]:
            stripped = line.strip()

            if not stripped:
                continue

            values = stripped.split()

            try:
                point_map = np.array([
                    float(values[x_index]),
                    float(values[y_index]),
                    float(values[z_index]),
                ], dtype=np.float64)
            except (ValueError, IndexError):
                continue

            if not np.all(np.isfinite(point_map)):
                continue

            point_local = rotation_local_map.dot(
                point_map - translation
            )

            values[x_index] = "{:.6f}".format(point_local[0])
            values[y_index] = "{:.6f}".format(point_local[1])
            values[z_index] = "{:.6f}".format(point_local[2])

            target.write(" ".join(values) + "\n")
            point_count += 1

    return point_count, rotation_local_map


def save_transform_record(
    output_path,
    translation_map_local,
    quaternion_map_local,
):
    """
    保存 nav_map -> map 静态 TF。

    nav_map 被定义为采集时刻的 local 水平坐标系。
    因此：

        T_nav_map_map = inverse(T_map_local)
    """
    rotation_map_local = quaternion_matrix(
        quaternion_map_local
    )[:3, :3]

    rotation_nav_map_map = rotation_map_local.T
    translation_nav_map_map = (
        -rotation_nav_map_map.dot(translation_map_local)
    )

    quaternion_nav_map_map = np.array([
        -quaternion_map_local[0],
        -quaternion_map_local[1],
        -quaternion_map_local[2],
        quaternion_map_local[3],
    ])

    rpy_map_local = euler_from_quaternion(
        quaternion_map_local
    )
    rpy_nav_map_map = euler_from_quaternion(
        quaternion_nav_map_map
    )

    with open(output_path, "w", encoding="utf-8") as file:
        file.write("# 自动生成的地图水平化变换记录\n")
        file.write("# nav_map 是重力对齐后的水平地图坐标系\n\n")

        file.write("map_to_local:\n")
        file.write(
            "  translation: [{:.9f}, {:.9f}, {:.9f}]\n".format(
                *translation_map_local
            )
        )
        file.write(
            "  quaternion_xyzw: [{:.9f}, {:.9f}, {:.9f}, {:.9f}]\n".format(
                *quaternion_map_local
            )
        )
        file.write(
            "  rpy_degree: [{:.6f}, {:.6f}, {:.6f}]\n\n".format(
                math.degrees(rpy_map_local[0]),
                math.degrees(rpy_map_local[1]),
                math.degrees(rpy_map_local[2]),
            )
        )

        file.write("nav_map_to_map:\n")
        file.write(
            "  translation: [{:.9f}, {:.9f}, {:.9f}]\n".format(
                *translation_nav_map_map
            )
        )
        file.write(
            "  quaternion_xyzw: [{:.9f}, {:.9f}, {:.9f}, {:.9f}]\n".format(
                *quaternion_nav_map_map
            )
        )
        file.write(
            "  rpy_degree: [{:.6f}, {:.6f}, {:.6f}]\n\n".format(
                math.degrees(rpy_nav_map_map[0]),
                math.degrees(rpy_nav_map_map[1]),
                math.degrees(rpy_nav_map_map[2]),
            )
        )

        file.write("# 后续静态 TF 命令模板：\n")
        file.write(
            "static_tf_command: "
            "rosrun tf2_ros static_transform_publisher "
            "{:.9f} {:.9f} {:.9f} "
            "{:.9f} {:.9f} {:.9f} {:.9f} "
            "nav_map map\n".format(
                translation_nav_map_map[0],
                translation_nav_map_map[1],
                translation_nav_map_map[2],
                quaternion_nav_map_map[0],
                quaternion_nav_map_map[1],
                quaternion_nav_map_map[2],
                quaternion_nav_map_map[3],
            )
        )


def main():
    parser = argparse.ArgumentParser(
        description="利用实时 map->local TF 将倾斜 PCD 转换到水平坐标系"
    )

    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--record", required=True)
    parser.add_argument("--parent", default="map")
    parser.add_argument("--child", default="local")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--rate", type=float, default=20.0)

    args = parser.parse_args()

    rospy.init_node("level_pcd_from_tf", anonymous=True)

    translation, quaternion = collect_transform(
        args.parent,
        args.child,
        args.samples,
        args.rate,
    )

    roll, pitch, yaw = euler_from_quaternion(quaternion)

    print()
    print("平均 map -> local：")
    print(
        "translation = [{:.6f}, {:.6f}, {:.6f}]".format(
            *translation
        )
    )
    print(
        "RPY degree  = [{:.3f}, {:.3f}, {:.3f}]".format(
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )
    )

    point_count, _ = transform_ascii_pcd(
        args.input,
        args.output,
        translation,
        quaternion,
    )

    save_transform_record(
        args.record,
        translation,
        quaternion,
    )

    print()
    print("水平化完成")
    print("输出点数：", point_count)
    print("水平点云：", args.output)
    print("变换记录：", args.record)


if __name__ == "__main__":
    main()
