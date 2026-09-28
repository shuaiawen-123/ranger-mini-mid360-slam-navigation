#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys


def percentile(sorted_values, percent):
    if not sorted_values:
        return float("nan")

    index = round((len(sorted_values) - 1) * percent / 100.0)
    return sorted_values[index]


def main():
    if len(sys.argv) != 2:
        print("用法: python3 pcd_z_stats.py 点云文件.pcd")
        sys.exit(1)

    filename = sys.argv[1]
    fields = None
    data_ascii = False
    z_values = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as file:
        for line in file:
            line = line.strip()

            if not data_ascii:
                if line.startswith("FIELDS "):
                    fields = line.split()[1:]

                elif line.startswith("DATA "):
                    data_type = line.split()[1].lower()

                    if data_type != "ascii":
                        print("错误：输入文件不是 ASCII PCD。")
                        sys.exit(1)

                    if fields is None or "z" not in fields:
                        print("错误：PCD 中找不到 z 字段。")
                        sys.exit(1)

                    z_index = fields.index("z")
                    data_ascii = True

                continue

            if not line:
                continue

            values = line.split()

            if len(values) <= z_index:
                continue

            try:
                z_values.append(float(values[z_index]))
            except ValueError:
                continue

    if not z_values:
        print("没有读取到有效的 Z 坐标。")
        sys.exit(1)

    z_values.sort()

    print("有效点数:", len(z_values))
    print()
    print("Z 高度统计（单位：m）")

    for p in [0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100]:
        print(f"{p:>3}% : {percentile(z_values, p): .4f}")


if __name__ == "__main__":
    main()
