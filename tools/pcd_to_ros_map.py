#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import sys

import numpy as np


UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def read_ascii_pcd(path):
    fields = None
    data_start_line = None

    with open(path, "r", encoding="utf-8", errors="ignore") as file:
        for line_number, line in enumerate(file):
            stripped = line.strip()

            if stripped.startswith("FIELDS "):
                fields = stripped.split()[1:]

            if stripped.lower() == "data ascii":
                data_start_line = line_number + 1
                break

    if fields is None:
        raise RuntimeError("PCD 文件中没有找到 FIELDS")

    if data_start_line is None:
        raise RuntimeError("输入文件必须是 DATA ascii 格式")

    for field in ("x", "y", "z"):
        if field not in fields:
            raise RuntimeError("PCD 中缺少字段：{}".format(field))

    x_index = fields.index("x")
    y_index = fields.index("y")
    z_index = fields.index("z")

    print("正在读取点云，请稍候……")

    data = np.loadtxt(
        path,
        skiprows=data_start_line,
        usecols=(x_index, y_index, z_index),
        dtype=np.float64,
    )

    if data.ndim == 1:
        data = data.reshape(1, 3)

    finite_mask = np.all(np.isfinite(data), axis=1)
    data = data[finite_mask]

    if len(data) == 0:
        raise RuntimeError("没有读取到有效点云")

    return data


def dilate_mask(mask, radius):
    """使用圆形邻域扩展二值栅格。"""
    radius = int(radius)

    if radius <= 0:
        return mask.copy()

    height, width = mask.shape
    result = mask.copy()

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue

            source_x0 = max(0, -dx)
            source_x1 = min(width, width - dx)
            source_y0 = max(0, -dy)
            source_y1 = min(height, height - dy)

            target_x0 = source_x0 + dx
            target_x1 = source_x1 + dx
            target_y0 = source_y0 + dy
            target_y1 = source_y1 + dy

            result[
                target_y0:target_y1,
                target_x0:target_x1
            ] |= mask[
                source_y0:source_y1,
                source_x0:source_x1
            ]

    return result


def add_points_to_grid(points, min_x, min_y, resolution, width, height):
    counts = np.zeros((height, width), dtype=np.uint16)

    if len(points) == 0:
        return counts

    x_indices = np.floor(
        (points[:, 0] - min_x) / resolution
    ).astype(np.int64)

    y_indices = np.floor(
        (points[:, 1] - min_y) / resolution
    ).astype(np.int64)

    valid = (
        (x_indices >= 0)
        & (x_indices < width)
        & (y_indices >= 0)
        & (y_indices < height)
    )

    x_indices = x_indices[valid]
    y_indices = y_indices[valid]

    np.add.at(counts, (y_indices, x_indices), 1)

    return counts


def write_pgm(path, image):
    height, width = image.shape

    # ROS 地图原点位于左下角，但 PGM 第一行位于图像顶部。
    output_image = np.flipud(image)

    with open(path, "wb") as file:
        file.write(
            "P5\n{} {}\n255\n".format(width, height).encode("ascii")
        )
        file.write(output_image.astype(np.uint8).tobytes())


def write_yaml(path, pgm_path, resolution, origin_x, origin_y):
    image_name = os.path.basename(pgm_path)

    with open(path, "w", encoding="utf-8") as file:
        file.write("image: {}\n".format(image_name))
        file.write("resolution: {:.6f}\n".format(resolution))
        file.write(
            "origin: [{:.6f}, {:.6f}, 0.000000]\n".format(
                origin_x,
                origin_y,
            )
        )
        file.write("negate: 0\n")
        file.write("occupied_thresh: 0.65\n")
        file.write("free_thresh: 0.196\n")


def main():
    parser = argparse.ArgumentParser(
        description="将水平化 ASCII PCD 转换为 ROS 二维栅格地图"
    )

    parser.add_argument("--input", required=True)
    parser.add_argument("--output-prefix", required=True)

    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--trim-percent", type=float, default=0.10)

    parser.add_argument("--floor-min", type=float, default=-0.60)
    parser.add_argument("--floor-max", type=float, default=-0.28)

    parser.add_argument("--obstacle-min", type=float, default=-0.20)
    parser.add_argument("--obstacle-max", type=float, default=1.50)

    parser.add_argument("--free-min-points", type=int, default=1)
    parser.add_argument("--obstacle-min-points", type=int, default=1)

    parser.add_argument("--free-expand", type=int, default=2)
    parser.add_argument("--occupied-expand", type=int, default=1)

    args = parser.parse_args()

    if args.resolution <= 0:
        print("错误：resolution 必须大于 0")
        sys.exit(1)

    points = read_ascii_pcd(args.input)
    z = points[:, 2]

    floor_mask = (
        (z >= args.floor_min)
        & (z <= args.floor_max)
    )

    obstacle_mask = (
        (z >= args.obstacle_min)
        & (z <= args.obstacle_max)
    )

    floor_points = points[floor_mask]
    obstacle_points = points[obstacle_mask]

    print("全部有效点数：", len(points))
    print("地面点数：", len(floor_points))
    print("障碍点数：", len(obstacle_points))

    selected = points[floor_mask | obstacle_mask]

    if len(selected) == 0:
        raise RuntimeError("所选高度范围内没有点")

    trim = max(0.0, min(10.0, args.trim_percent))

    min_x_raw = np.percentile(selected[:, 0], trim)
    max_x_raw = np.percentile(selected[:, 0], 100.0 - trim)
    min_y_raw = np.percentile(selected[:, 1], trim)
    max_y_raw = np.percentile(selected[:, 1], 100.0 - trim)

    min_x = math.floor(
        (min_x_raw - args.padding) / args.resolution
    ) * args.resolution

    max_x = math.ceil(
        (max_x_raw + args.padding) / args.resolution
    ) * args.resolution

    min_y = math.floor(
        (min_y_raw - args.padding) / args.resolution
    ) * args.resolution

    max_y = math.ceil(
        (max_y_raw + args.padding) / args.resolution
    ) * args.resolution

    width = int(round((max_x - min_x) / args.resolution)) + 1
    height = int(round((max_y - min_y) / args.resolution)) + 1

    if width <= 0 or height <= 0:
        raise RuntimeError("地图尺寸计算失败")

    if width * height > 100_000_000:
        raise RuntimeError(
            "地图尺寸过大：{} × {}，请增大 resolution 或 trim-percent".format(
                width,
                height,
            )
        )

    print()
    print("地图范围：")
    print("  X: {:.2f} ～ {:.2f} m".format(min_x, max_x))
    print("  Y: {:.2f} ～ {:.2f} m".format(min_y, max_y))
    print("  分辨率: {:.3f} m/cell".format(args.resolution))
    print("  尺寸: {} × {} cells".format(width, height))

    floor_counts = add_points_to_grid(
        floor_points,
        min_x,
        min_y,
        args.resolution,
        width,
        height,
    )

    obstacle_counts = add_points_to_grid(
        obstacle_points,
        min_x,
        min_y,
        args.resolution,
        width,
        height,
    )

    free_mask = floor_counts >= args.free_min_points
    occupied_mask = obstacle_counts >= args.obstacle_min_points

    free_mask = dilate_mask(free_mask, args.free_expand)
    occupied_mask = dilate_mask(
        occupied_mask,
        args.occupied_expand,
    )

    image = np.full(
        (height, width),
        UNKNOWN,
        dtype=np.uint8,
    )

    image[free_mask] = FREE

    # 障碍物优先级高于可通行地面。
    image[occupied_mask] = OCCUPIED

    output_prefix = args.output_prefix
    output_directory = os.path.dirname(output_prefix)

    if output_directory:
        os.makedirs(output_directory, exist_ok=True)

    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"

    write_pgm(pgm_path, image)
    write_yaml(
        yaml_path,
        pgm_path,
        args.resolution,
        min_x,
        min_y,
    )

    total_cells = width * height
    free_cells = int(np.count_nonzero(image == FREE))
    occupied_cells = int(np.count_nonzero(image == OCCUPIED))
    unknown_cells = int(np.count_nonzero(image == UNKNOWN))

    print()
    print("二维地图生成完成：")
    print("  PGM :", pgm_path)
    print("  YAML:", yaml_path)
    print()
    print("栅格统计：")
    print(
        "  可通行：{} ({:.2f}%)".format(
            free_cells,
            100.0 * free_cells / total_cells,
        )
    )
    print(
        "  障碍物：{} ({:.2f}%)".format(
            occupied_cells,
            100.0 * occupied_cells / total_cells,
        )
    )
    print(
        "  未知区域：{} ({:.2f}%)".format(
            unknown_cells,
            100.0 * unknown_cells / total_cells,
        )
    )


if __name__ == "__main__":
    main()
