# Ranger Mini + Livox MID360
# 3D SLAM, Relocalization and Autonomous Navigation System


## 1. Project Overview

本项目基于 Ranger Mini 移动机器人平台，
集成 Livox MID360 激光雷达，
实现三维激光建图、基于先验地图的重定位、
二维导航以及任务层控制。


主要功能：

- FAST-LIO 三维点云建图
- FASTLIO2-SAM-LC 激光重定位
- nav_map / map / odom 坐标转换
- move_base + TEB自主导航
- 往返任务控制


---

## 2. Hardware

Robot:
- Ranger Mini mobile robot

Sensor:
- Livox MID360 LiDAR


---

## 3. Software Environment

- Ubuntu 22.04
- Docker
- ROS Noetic


---

## 4. System Architecture


Livox MID360

↓

FAST-LIO

↓

3D Point Cloud Map

↓

FASTLIO2-SAM-LC

↓

Relocalization

↓

TF Coordinate Bridge

↓

move_base + TEB Planner

↓

Ranger Mini


---

## 5. ROS Packages


### FAST_LIO

三维激光SLAM建图。


### FASTLIO2_SAM_LC

基于已有点云地图进行定位。


### ranger_localization_bridge

负责地图坐标系与导航坐标系转换。


### ranger_navigation

负责二维地图导航。


### ranger_round_trip

任务层控制，包括：

- 自动导航任务
- cmd_vel速度仲裁


---

## 6. Map Structure


maps/

保存三维点云地图。


navigation_maps/

保存二维导航地图。


datasets/

保存实验采集数据。


---

## 7. Main Launch


roslaunch ranger_navigation full_navigation.launch


启动流程：

1. Ranger底盘启动
2. MID360驱动启动
3. FASTLIO2-SAM-LC重定位
4. 地图加载
5. 导航启动


---

## 8. Future Work

后续研究方向：

- 机器人运动控制
- MPC/WBC控制方法
- 强化学习控制策略