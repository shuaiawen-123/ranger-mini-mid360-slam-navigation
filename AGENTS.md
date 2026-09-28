# 项目环境

- 主机系统：Ubuntu 22.04
- 主机 ROS：ROS2 Humble
- 当前开发环境：Docker Dev Container
- 容器系统：Ubuntu 20.04
- 容器 ROS：ROS1 Noetic
- 工作空间：/home/shuaiawen/ROS1/slam/slam_mapping_ws
- 雷达：Livox MID360
- 雷达驱动：livox_ros_driver2
- 建图算法：FAST-LIO2
- 最终目标：建图、PCD 地图保存、历史地图重定位、ROS1 导航

# 操作规则

1. 所有 ROS1 命令必须在 Noetic 容器中执行。
2. 不要使用主机 ROS2 Humble 的环境。
3. 编译前执行：
   source /opt/ros/noetic/setup.bash
4. 编译命令优先使用：
   catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j1
5. 不要修改 build 和 devel 中的生成文件。
6. 不要删除源码、地图、配置文件和 rosbag。
7. 修改文件前先说明原因，并展示计划。
8. 修改后必须重新编译并检查第一个真实 error。
9. MID360 使用 livox_ros_driver2/CustomMsg。
10. 建图算法使用 FAST-LIO2，不使用 Point-LIO。