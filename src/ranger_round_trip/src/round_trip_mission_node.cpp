/*-------第一部分：程序启动阶段--------*/
//1. 加载ROS功能模块

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>// 提供Action客户端，本程序通过它连接move_base
#include <geometry_msgs/PoseStamped.h>//带坐标系的机器人位姿
#include <geometry_msgs/Twist.h>//机器人速度
#include <move_base_msgs/MoveBaseAction.h>//导航核心
//2. 加载TF坐标转换模块

#include <tf2/LinearMath/Quaternion.h>//ROS姿态使用：Quaternion
#include <tf2/utils.h>//四元数转换
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//tf2和类型geometry_msgs类型互相转换
#include <tf2_ros/transform_listener.h>//获得机器人在地图中的位置

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>

namespace
{
    //定义圆周率
constexpr double kPi = 3.14159265358979323846;
/*----------第二部分：辅助数学函数---------*/
//3. 角度归一化函数 normalizeAngle()

//// 将任意角度转换到[-π,π]范围，这是机器人旋转控制中非常重要的函数，因为机器人角度具有周期性，不处理会导致旋转方向错误。
double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
//4. 速度限制函数 clampValue()

//限制输入值范围
double clampValue(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(value, maximum));
}
}  // namespace
/*----------第三部分：RoundTripMission类初始化------------*/
//5. 创建任务控制类

// 定义整个往返任务控制类，所有机器人任务逻辑都封装在这里，
//包括：接收目标点、调用move_base导航、读取TF定位、控制原地旋转、管理任务状态。
class RoundTripMission
{
//定义move_base客户端类型
public:
    using MoveBaseClient =
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
//6. 构造函数初始化系统

    // 构造函数在节点启动时自动执行，负责完成机器人任务节点初始化。
    RoundTripMission()
        : nh_(),// 创建ROS全局NodeHandle，用于访问全局命名空间。
          pnh_("~"),// 创建私有NodeHandle，用于读取当前节点私有参数。
          tf_listener_(tf_buffer_),// 创建TF监听器，并绑定TF缓存区。TF监听器会不断接收ROS TF树数据，然后保存到tf_buffer_中，后续程序通过lookupTransform查询机器人位置。
          mission_running_(false)// 初始化任务运行状态为false，表示当前没有执行往返任务。
    {

//7. 读取launch参数  

        //从ROS参数服务器读取任务目标点话题名称，如果launch没有配置，则默认使用/round_trip_goal。
        pnh_.param<std::string>(
            "goal_topic",  //接收任务目标
            goal_topic_,
            "/round_trip_goal");//接收用户发送的终点

        // 保存move_base Action服务器名称，后面创建Action客户端时使用。
        pnh_.param<std::string>(
            "action_name",  //连接move_base
            action_name_,
            "/move_base");

        // 设置全局地图坐标系，本项目采用nav_map作为导航地图坐标系。
        pnh_.param<std::string>(
            "global_frame", //地图坐标
            global_frame_,
            "nav_map");

        // 设置机器人局部里程计坐标系，主要用于原地旋转控制。
        pnh_.param<std::string>(
            "odom_frame",  //旋转参考
            odom_frame_,
            "odom");

        // 设置机器人本体坐标系，TF查询最终都是查询base_link的位置。
        pnh_.param<std::string>(
            "base_frame",  //机器人坐标系
            base_frame_,
            "base_link");

        pnh_.param<std::string>(
            "cmd_vel_topic",  //速度输出
            cmd_vel_topic_,
            "/cmd_vel");

        pnh_.param(
            "wait_after_end_turn",
            wait_after_end_turn_,
            0.5);

        pnh_.param(
            "settle_time",
            settle_time_,
            0.3);

        pnh_.param(
            "goal_timeout",  //导航超时
            goal_timeout_,
            180.0);

        pnh_.param(
            "minimum_route_distance",
            minimum_route_distance_,
            0.5);

        /*
         * 返程阶段只检查位置。
         * 进入该距离后取消 move_base，
         * 随后由阶段4直接执行原地掉头。
         */
        pnh_.param(
            "return_position_tolerance",  //回到起点附近的距离阈值
            return_position_tolerance_,
            0.35);

        // 原地旋转参数
        pnh_.param(
            "turn_direction",  //顺逆时针
            turn_direction_,
            1);

        pnh_.param(
            "turn_angle",  //旋转角度
            turn_angle_,
            kPi);

        pnh_.param(
            "turn_max_speed",  //最大旋转速度
            turn_max_speed_,
            0.15);

        pnh_.param(
            "turn_min_speed",  //最小旋转速度
            turn_min_speed_,
            0.05);

        pnh_.param(
            "turn_kp",  //旋转P控制增益
            turn_kp_,
            0.8);

        pnh_.param(
            "turn_tolerance",  //旋转停止误差
            turn_tolerance_,
            0.08);

        pnh_.param(
            "turn_timeout",  //旋转超时
            turn_timeout_,
            35.0);

        pnh_.param(
            "turn_rate",
            turn_rate_,
            20.0);

        pnh_.param(
            "stop_hold_time",
            stop_hold_time_,
            0.5);

        turn_direction_ =
            (turn_direction_ >= 0) ? 1 : -1; //顺逆时针

//8. 创建move_base客户端

        // 创建一个move_base客户端对象，用于向导航系统发送目标点。
        //这里使用unique_ptr智能指针管理对象生命周期，避免手动new/delete造成内存泄漏。
        move_base_client_ =
            std::make_unique<MoveBaseClient>(
                action_name_,
                true);//Action客户端自己开启线程处理通信。

//9. 创建cmd_vel发布器

        // 创建速度发布器，向cmd_vel_topic_发送Twist速度指令。主要用于原地旋转阶段，由本节点直接控制底盘。
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(//原地掉头时使用的速度。
            cmd_vel_topic_,
            10);
        // 订阅往返任务目标点，当RViz或者其他节点发布目标点后，会调用missionGoalCallback函数启动一次完整任务。

//10. 创建任务目标订阅

        mission_goal_sub_ = nh_.subscribe(
            goal_topic_,
            1,
            &RoundTripMission::missionGoalCallback, //回调函数地址。
            this);

        // 节点启动时打印当前配置，方便调试机器人系统。ROS机器人项目中大量使用启动日志，因为实际运行时无法直接观察内部变量。
        ROS_INFO_STREAM(
            "\nRound-trip mission node started"
            << "\n  goal topic: " << goal_topic_
            << "\n  move_base action: " << action_name_
            << "\n  global frame: " << global_frame_
            << "\n  turn reference frame: " << odom_frame_
            << "\n  base frame: " << base_frame_
            << "\n  turn speed: " << turn_max_speed_
            << " rad/s"
            << "\n  turn tolerance: "
            << turn_tolerance_ << " rad");
    }

    // 析构函数，在节点退出时自动执行，用于安全关闭机器人相关资源。
    ~RoundTripMission()
    {
        if (move_base_client_)
        {
            move_base_client_->cancelAllGoals();// 如果move_base客户端存在，则取消所有正在执行的导航任务。
        }

        publishZeroVelocity(0.3);// 发布0速度持续0.3秒，让底盘确认停止。
    }

private:
    // 给一个PoseStamped设置目标方向，只修改yaw角，不改变位置。
    static void setYaw(
        geometry_msgs::PoseStamped& pose,
        double yaw)
    {
        tf2::Quaternion quaternion;// ROS中姿态不能直接保存yaw，需要转换成四元数。
        // 将欧拉角转换成四元数。移动机器人一般只在二维平面运动，因此roll和pitch固定为0，只改变yaw。
        quaternion.setRPY(
            0.0,//roll
            0.0,//pitch
            normalizeAngle(yaw));
        // 将TF2内部Quaternion转换成ROS消息Quaternion，使其可以放入PoseStamped。
        pose.pose.orientation =
            tf2::toMsg(quaternion);
    }
    //在执行导航任务之前，确认 move_base 导航服务器已经启动，否则发送目标点会失败。
    bool waitForMoveBase()
    {
        // 只要ROS系统没有关闭，就持续尝试连接move_base。
        while (ros::ok())
        {
            // 等待move_base Action服务器出现，最多等待1秒。如果服务器连接成功，返回true。
            if (move_base_client_->waitForServer(
                    ros::Duration(1.0)))
            {
                // move_base连接成功，返回true，允许后续执行任务。
                ROS_INFO("Connected to move_base.");
                return true;
            }
            // 每5秒打印一次等待信息，避免循环频繁刷屏。
            ROS_WARN_THROTTLE(
                5.0,
                "Waiting for move_base...");
        }

        return false;
    }

//11. 获取机器人当前位置

    //TF树其实就是这个任务程序获取机器人状态的重要“传感器”。
    geometry_msgs::PoseStamped getRobotPose(
        const std::string& target_frame)
    {
        // 从TF缓存中查询target_frame到base_link的变换关系，例如查询nav_map->base_link，得到机器人在地图中的位置和姿态。
        const geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform(
                target_frame,//我要在哪个坐标系下描述机器人。
                base_frame_, //机器人本体。
                ros::Time(0),//获取最新TF
                ros::Duration(2.0));//最多等待2秒，若TF不存在2s后报错

        // 创建一个ROS标准位姿消息，用于保存机器人当前位置。
        geometry_msgs::PoseStamped pose;
        // 标明当前机器人位姿属于哪个坐标系。
        pose.header.frame_id = target_frame;
        // 记录当前时间戳，ROS消息通常需要时间信息，方便其他节点进行同步。
        pose.header.stamp = ros::Time::now();
        // 从TF结果中读取机器人X坐标。
        pose.pose.position.x =
            transform.transform.translation.x;
        // 保存机器人Y方向位置。
        pose.pose.position.y =
            transform.transform.translation.y;
        // 保存高度信息，虽然Ranger Mini是二维移动机器人，但TF统一使用三维坐标。
        pose.pose.position.z =
            transform.transform.translation.z;
        // 保存机器人当前四元数姿态。
        pose.pose.orientation =
            transform.transform.rotation;
        // 返回机器人当前完整状态。
        return pose;
    }

//12. 获取机器人yaw角    
    // 获取机器人当前朝向角，只提取yaw角，用于原地旋转控制。因为Ranger Mini属于二维移动机器人，控制旋转时只关心绕Z轴的偏航角yaw，而不需要roll和pitch。
    bool getRobotYaw(
        const std::string& target_frame,
        double& yaw)
    {
        try
        {
            // 查询机器人base_link相对于target_frame的坐标变换，例如查询odom->base_link，得到机器人当前的位置和姿态。
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame,//odom
                    base_frame_,
                    ros::Time(0),
                    ros::Duration(1.0));
            // 将TF中的四元数姿态转换成yaw角。
            yaw = tf2::getYaw(
                transform.transform.rotation);

            return true;
        }
        // 捕获TF查询失败异常，例如TF树断开、坐标系不存在、数据超时。
        catch (const tf2::TransformException& error)
        {
            // 每2秒输出一次TF读取失败提示，避免循环控制过程中疯狂刷屏。
            ROS_WARN_STREAM_THROTTLE(
                2.0,
                "Unable to read "
                << target_frame
                << " -> "
                << base_frame_
                << ": "
                << error.what());

            return false;
        }
    }
/*------------第五部分：目标处理模块------------*/
//13. 目标点坐标转换    
    // 将用户输入的目标点转换到机器人导航统一坐标系global_frame_。
    geometry_msgs::PoseStamped transformGoal(
        // 创建输入目标的副本，避免直接修改用户传入的数据。
        const geometry_msgs::PoseStamped& input_goal)
    {
        geometry_msgs::PoseStamped source = input_goal;
        // 如果目标点没有指定坐标系，默认认为目标点已经属于导航地图坐标系。
        if (source.header.frame_id.empty())
        {
            source.header.frame_id = global_frame_;
        }
        // 设置时间为0，表示请求TF最新变换。
        source.header.stamp = ros::Time(0);
        // 如果目标点本身已经在nav_map坐标系，不需要转换，直接返回。
        if (source.header.frame_id == global_frame_)
        {
            source.header.stamp = ros::Time::now();
            return source;
        }

        geometry_msgs::PoseStamped transformed;
        // 使用TF将目标点从原坐标系转换到global_frame_。
        tf_buffer_.transform(
            source,
            transformed,
            global_frame_,
            ros::Duration(2.0));

        transformed.header.stamp = ros::Time::now();
        return transformed;
    }

/*----------第六部分：导航控制模块-----------*/

    // 发布零速度，让机器人停止。
    void publishZeroVelocity(double duration)
    {
        geometry_msgs::Twist zero_command;// 默认初始化为0。

        ros::WallRate rate(20.0);// 以20Hz频率发送停止指令。
        // 计算停止指令需要持续多久。
        const ros::WallTime end_time =
            ros::WallTime::now() +
            ros::WallDuration(std::max(0.0, duration));
        // 在时间结束之前持续发布停止命令。
        do
        {
            cmd_vel_pub_.publish(zero_command);
            rate.sleep();
        }
        while (
            ros::ok() &&
            ros::WallTime::now() < end_time);
    }
//16. 单目标导航
    // 这是第一次真正调用move_base导航的函数。
    bool sendGoalAndWait(
        geometry_msgs::PoseStamped target_pose,
        const std::string& stage_name)
    {
        // 强制目标点使用统一导航坐标系nav_map。
        target_pose.header.frame_id = global_frame_;
        target_pose.header.stamp = ros::Time::now();
        // 将普通PoseStamped包装成move_base需要的Action Goal格式。
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = target_pose;

        ROS_INFO_STREAM(
            stage_name
            << "\n  x = "
            << target_pose.pose.position.x
            << "\n  y = "
            << target_pose.pose.position.y
            << "\n  yaw = "
            << tf2::getYaw(
                   target_pose.pose.orientation));

        // 向move_base发送目标点，之后由TEB负责路径规划和局部控制。
        move_base_client_->sendGoal(goal);

        bool finished = false;

        if (goal_timeout_ <= 0.0)
        {
            move_base_client_->waitForResult();
            finished = true;
        }
        else
        {
            // 等待move_base执行完成，超过180秒认为失败。
            finished =
                move_base_client_->waitForResult(
                    ros::Duration(goal_timeout_));
        }
        // 如果导航超过规定时间，取消目标并停止机器人。
        if (!finished)
        {
            ROS_ERROR_STREAM(
                stage_name << " timed out.");

            move_base_client_->cancelGoal();
            publishZeroVelocity(0.5);
            return false;
        }
        // 获取move_base执行结果。
        const actionlib::SimpleClientGoalState state =
            move_base_client_->getState();

        if (state !=
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR_STREAM(
                stage_name
                << " failed. State: "
                << state.toString());

            publishZeroVelocity(0.5);
            return false;
        }

        ROS_INFO_STREAM(stage_name << " completed.");
        return true;
    }
//17. 返回导航控制
    //返回起点时，不要求 move_base 完成最终姿态调整。
    //位置够不够近
    bool sendReturnGoalUntilPositionReached(
        geometry_msgs::PoseStamped target_pose,
        const std::string& stage_name)
    {
        // 设置返程目标点坐标系为nav_map，保证move_base使用正确地图坐标。
        target_pose.header.frame_id = global_frame_;
        target_pose.header.stamp = ros::Time::now();
        // 将返回目标包装成move_base Action目标。
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = target_pose;

        ROS_INFO_STREAM(
            stage_name
            << "\n  x = "
            << target_pose.pose.position.x
            << "\n  y = "
            << target_pose.pose.position.y
            << "\n  reference yaw = "
            << tf2::getYaw(
                   target_pose.pose.orientation)
            << "\n  position tolerance = "
            << return_position_tolerance_
            << " m");
        // 让TEB开始执行返程路径规划。
        move_base_client_->sendGoal(goal);
        // 记录任务开始时间，用于超时判断。
        const ros::WallTime start_time =
            ros::WallTime::now();

         ros::WallRate rate(10.0);// 以10Hz频率检查机器人状态。
        // 调用前面分析过的TF函数，获取机器人当前在nav_map中的位置。
        while (ros::ok())
        {
            try
            {
                const geometry_msgs::PoseStamped current_pose =
                    getRobotPose(global_frame_);
                // 计算目标点和机器人当前位置的x方向误差。
                const double dx =
                    target_pose.pose.position.x -
                    current_pose.pose.position.x;
                // 计算y方向误差
                const double dy =
                    target_pose.pose.position.y -
                    current_pose.pose.position.y;
                // 使用欧氏距离计算机器人距离目标点还有多远。

                const double distance =
                    std::hypot(dx, dy);

                ROS_INFO_STREAM_THROTTLE(
                    1.0,
                    stage_name
                    << ": distance to start = "
                    << distance << " m");

                /*
                 * 只要位置已经接近起点，就立即结束 TEB 控制。
                 * 不再等待 TEB 调整最终航向。
                 */
                // 判断机器人是否进入起点附近区域。
                if (distance <= return_position_tolerance_)
                {
                    ROS_INFO_STREAM(
                        stage_name
                        << ": start position reached. "
                        << "Canceling move_base before final turn.");
                    // 取消TEB当前导航任务。
                    move_base_client_->cancelGoal();
                    // 等待move_base完成取消动作。
                    move_base_client_->waitForResult(
                        ros::Duration(0.5));
                    // 取消导航后发送0速度，保证底盘停止。
                    publishZeroVelocity(0.5);

                    ROS_INFO_STREAM(
                        stage_name
                        << " completed by position.");
                    // 返回阶段完成，后续进入原地旋转。
                    return true;
                }
            }
            catch (const tf2::TransformException& error)
            {
                ROS_WARN_STREAM_THROTTLE(
                    2.0,
                    stage_name
                    << ": unable to read robot pose: "
                    << error.what());
            }
            // 查询move_base当前状态。
            const actionlib::SimpleClientGoalState state =
                move_base_client_->getState();

            /*
             * move_base 提前结束时进行检查。
             * 如果还没有进入位置容差，就认为返程失败。
             */
            if (state.isDone()) //move_base已经结束。
            {
                if (state ==
                    actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    publishZeroVelocity(0.5);

                    ROS_INFO_STREAM(
                        stage_name
                        << " completed by move_base.");

                    return true;
                }

                ROS_ERROR_STREAM(
                    stage_name
                    << " ended before reaching start position. "
                    << "State: "
                    << state.toString());

                publishZeroVelocity(0.5);
                return false;
            }

            if (goal_timeout_ > 0.0) // 计算已经运行多少秒。
            {
                const double elapsed =
                    (ros::WallTime::now() -
                     start_time).toSec();

                if (elapsed > goal_timeout_)
                {
                    ROS_ERROR_STREAM(
                        stage_name
                        << " timed out after "
                        << elapsed << " seconds.");
                    
                    move_base_client_->cancelGoal();
                    publishZeroVelocity(0.5);
                    return false;
                }
            }

            rate.sleep();
        }
        // 取消任务并停止机器人。
        move_base_client_->cancelGoal();
        publishZeroVelocity(0.5);

        return false;
    }
/*---------------第七部分：旋转控制模块---------------*/
//18. 原地180°旋转控制
    //自己做一个小型闭环角度控制器。
    // 执行机器人原地旋转180度，返回true表示旋转成功，false表示失败。
    bool rotateInPlace180(
        const std::string& stage_name)
    {
        // 提示当前阶段由导航控制切换到自主旋转控制。
        ROS_INFO_STREAM(
            stage_name
            << ": canceling move_base and starting direct odom-based turn.");

//19. 取消move_base控制权
         move_base_client_->cancelAllGoals();// 取消move_base当前所有导航任务。

        ros::WallDuration(0.5).sleep();// 给move_base半秒时间完成退出。
        publishZeroVelocity(0.3);// 确保机器人完全停止后再开始旋转。
        //定义上一时刻角度。
        double previous_yaw = 0.0;
//20. 获取初始yaw
        // 获取机器人开始旋转时的odom yaw。
        if (!getRobotYaw(
                odom_frame_,
                previous_yaw))
        {
            ROS_ERROR(
                "Cannot obtain initial odom yaw.");
            return false;
        }
//21. 角度累计
        // 保存机器人已经旋转了多少角度。
        double accumulated_angle = 0.0;

        const ros::WallTime start_time =
            ros::WallTime::now();

        ros::WallRate rate(
            std::max(5.0, turn_rate_));

        while (ros::ok())
        {   
            // 当前旋转已经执行时间。
            const double elapsed =
                (ros::WallTime::now() -
                 start_time).toSec();

            if (elapsed > turn_timeout_)
            {
                ROS_ERROR_STREAM(
                    stage_name
                    << " timed out after "
                    << elapsed << " seconds.");

                publishZeroVelocity(stop_hold_time_);
                return false;
            }

            // 获取机器人当前角度。
            double current_yaw = 0.0;

            if (!getRobotYaw(
                    odom_frame_,
                    current_yaw))
            {
                publishZeroVelocity(0.1);
                rate.sleep(); //继续尝试。
                continue;
            }

            //计算角度变化
            const double delta_yaw =
                normalizeAngle(
                    current_yaw - previous_yaw);
                    
            // 保存当前角度，并累计本次旋转量。
            previous_yaw = current_yaw;
            accumulated_angle += delta_yaw;

            //判断旋转方向
            const double progress =
                static_cast<double>(turn_direction_) *
                accumulated_angle;
            //计算剩余角度=目标角度−已经旋转角度
            const double remaining =
                turn_angle_ - progress;

//22. 计算角度误差
            if (remaining <= turn_tolerance_)
            {
                publishZeroVelocity(stop_hold_time_);
                
                //实际旋转角度。
                ROS_INFO_STREAM(
                    stage_name
                    << " completed. Rotated "
                    << progress * 180.0 / kPi
                    << " degrees.");

                return true;
            }

//23. P控制计算旋转速度
            //P控制器部分
            const double angular_speed =
//24. 速度限制
            clampValue(   //速度限制
                    turn_kp_ * remaining,
                    turn_min_speed_,
                    turn_max_speed_);
            // 构造纯旋转速度。
            geometry_msgs::Twist command;
            command.linear.x = 0.0;
            command.linear.y = 0.0;
            command.angular.z =
                static_cast<double>(turn_direction_) *
                angular_speed;

//25. 发布旋转速度
                cmd_vel_pub_.publish(command);

            ROS_INFO_STREAM_THROTTLE(
                1.0,
                stage_name
                << ": progress = "
                << progress * 180.0 / kPi
                << " deg, remaining = "
                << remaining * 180.0 / kPi
                << " deg, command = "
                << command.angular.z);

            rate.sleep();
        }

        publishZeroVelocity(stop_hold_time_);
        return false;
    }

/*--------第八部分：任务状态机----------*/    
//26. 路径合法性检查
    // 判断起点和终点之间的距离是否满足任务要求，防止机器人执行无意义的小范围往返任务。
    bool validateRoute(
        const geometry_msgs::PoseStamped& start_pose,
        const geometry_msgs::PoseStamped& end_pose)
    {
        // 计算目标点和起点在x方向上的位置差。
        const double dx =
            end_pose.pose.position.x -
            start_pose.pose.position.x;

        // 计算目标点和起点在y方向上的位置差。
        const double dy =
            end_pose.pose.position.y -
            start_pose.pose.position.y;

        // 使用二维欧氏距离计算两点之间的直线距离。
        const double distance =
            std::hypot(dx, dy);

        // 判断目标点是否离机器人太近。
        if (distance < minimum_route_distance_)
        {
            // 输出任务距离不足的信息。
            ROS_ERROR_STREAM(
                "Route distance "
                << distance
                << " m is less than minimum "
                << minimum_route_distance_
                << " m.");
            // 告诉上层任务管理函数，不执行后续流程。
            return false;
        }

        ROS_INFO_STREAM(
            "One-way route distance: "
            << distance << " m.");

        return true;
    }
//27. 总任务执行函数
    void runMission(
        const geometry_msgs::PoseStamped& received_goal)
    {
        // 确认导航系统已经启动，否则不执行任务。
        if (!waitForMoveBase())
        {
            return;
        }
        // 保存任务开始位置和最终目标位置。
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped end_goal;

        try
        {
            // 获取机器人当前在nav_map中的位置。
            start_pose =
                getRobotPose(global_frame_);
            // 将用户发送的目标点转换到统一导航坐标系。
            end_goal =
                transformGoal(received_goal);
        }
        // 如果TF转换失败，任务直接退出。
        catch (const tf2::TransformException& error)
        {
            ROS_ERROR_STREAM(
                "Unable to acquire mission poses: "
                << error.what());

            return;
        }
        // 路径检查。
        if (!validateRoute(
                start_pose,
                end_goal))
        {
            return;
        }
        // 提取机器人初始朝向。
        const double start_yaw =
            tf2::getYaw(
                start_pose.pose.orientation);
        // 打印任务起点。
        ROS_INFO_STREAM(
            "Start pose recorded:"
            << "\n  x = "
            << start_pose.pose.position.x
            << "\n  y = "
            << start_pose.pose.position.y
            << "\n  yaw = "
            << start_yaw);

//Stage 1   // 阶段1：导航去终点
        if (!sendGoalAndWait(
                end_goal,
                "Stage 1/4: drive to endpoint"))
        {
            return;
        }
        ros::WallDuration(settle_time_).sleep();

//Stage 2   // 阶段2：直接基于 odom 原地旋转180°
        if (!rotateInPlace180(
                "Stage 2/4: endpoint 180-degree turn"))
        {
            return;
        }
        ROS_INFO_STREAM(
            "Endpoint turn completed. Waiting "
            << wait_after_end_turn_
            << " seconds.");
        //终点旋转后等待
        publishZeroVelocity(wait_after_end_turn_);
        // 返回目标首先设置成任务开始位置。
        geometry_msgs::PoseStamped return_goal =
            start_pose;
        try
        {
            const geometry_msgs::PoseStamped current_pose =
                getRobotPose(global_frame_);
            //计算返回方向
            const double return_heading =
                std::atan2(
                    start_pose.pose.position.y -
                    current_pose.pose.position.y,
                    start_pose.pose.position.x -
                    current_pose.pose.position.x);
            setYaw(
                return_goal,
                return_heading);
        }
        catch (const tf2::TransformException& error)
        {
            ROS_ERROR_STREAM(
                "Unable to calculate return heading: "
                << error.what());
            return;
        }
//Stage 3      // 返回起点
        if (!sendReturnGoalUntilPositionReached(
                return_goal,
                "Stage 3/4: return to start"))
        {
            return;
        }
        ros::WallDuration(settle_time_).sleep();
//Stage 4      // 阶段4：在起点再次直接旋转180°
        if (!rotateInPlace180(
                "Stage 4/4: start-point 180-degree turn"))
        {
            return;
        }

        publishZeroVelocity(0.5);
        //任务完成
        ROS_INFO(
            "Round-trip mission completed successfully.");
    }
/*-----------第九部分：任务入口-------------*/
//28. 接收任务目标
    // ROS订阅回调函数，当收到新的往返任务目标点时执行
    void missionGoalCallback(
        const geometry_msgs::PoseStamped::ConstPtr& message)
    {
//29. 防止重复任务
        if (mission_running_.exchange(true))
        {
            ROS_WARN(
                "A mission is already running. "
                "The new goal is ignored.");
            return;
        }
        // 输出收到新的往返任务。
        ROS_INFO("New round-trip endpoint received.");

        try
        {
            runMission(*message);
        
        }
        // 捕获任务执行过程中出现的异常。TF异常，Action异常，逻辑错误
        catch (const std::exception& error)
        {
            ROS_ERROR_STREAM(
                "Mission exception: "
                << error.what());
            // 如果任务异常，取消导航并发送停止速度
            move_base_client_->cancelAllGoals();
            publishZeroVelocity(0.5);
        }
        // 标记当前任务已经结束，可以接受新的目标。
        mission_running_.store(false);
        // 提示任务系统重新进入空闲状态。
        ROS_INFO(
            "Mission node is ready for the next endpoint.");
    }

private:
    //管理ROS通信。
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    //TF相关：机器人定位核心。
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    //move_base客户端发送：目标点，接收：导航结果
    std::unique_ptr<MoveBaseClient>
        move_base_client_;
    //ROS通信
    ros::Subscriber mission_goal_sub_;
    ros::Publisher cmd_vel_pub_;
    //是否允许新任务。
    std::atomic<bool> mission_running_;
    
    std::string goal_topic_;
    std::string action_name_;
    //坐标系变量
    std::string global_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string cmd_vel_topic_;

    double wait_after_end_turn_;
    double settle_time_;
    double goal_timeout_;
    double minimum_route_distance_;
    double return_position_tolerance_;

    int turn_direction_;
    //旋转控制参数
    double turn_angle_;//目标角度
    double turn_max_speed_;//最大角速度
    double turn_min_speed_;//最小角速度
    double turn_kp_;//比例系数
    double turn_tolerance_;
    double turn_timeout_;
    double turn_rate_;
    double stop_hold_time_;
};
/*--------------第十部分：程序入口-------------*/
//30. main函数
//ROS节点入口
int main(int argc, char** argv)
{
    //初始化ROS
    ros::init(
        argc,
        argv,
        "round_trip_mission");

    RoundTripMission mission;//创建任务对象

    ros::AsyncSpinner spinner(3); //三个线程ROS处理。
    spinner.start();

    ros::waitForShutdown();// 保持节点运行直到Ctrl+C。

    return 0;
}
