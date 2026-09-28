#include <ros/ros.h>
#include "fastlio/SlamReLoc.h"  // 自动生成的服务头文件
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class init_pose 
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::ServiceClient reloc_client_;
    std::string map_path_;

public:
    init_pose() 
    {
        map_path_ = ""; //  无用
        
        // 初始化服务客户端
        reloc_client_ = nh_.serviceClient<fastlio::SlamReLoc>("slam_reloc");
        ROS_INFO("Waiting for service to become available...");
        // ros::service::waitForService("slam_reloc");
        if (!reloc_client_.waitForExistence(ros::Duration(5.0))) {
            ROS_WARN("Service connection timeout, will keep trying");
        }
        
        // 订阅初始位姿话题
        pose_sub_ = nh_.subscribe("/initialpose", 10, &init_pose::poseCallback, this);
        ROS_INFO("Subscribed to /initialpose topic");
        ROS_INFO("waiting for initial pose data...");
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg) 
    {
        ROS_INFO("Received new initial pose data");
        
        // 提取位置信息
        const float x = _msg->pose.pose.position.x;
        const float y = _msg->pose.pose.position.y;
        const float z = _msg->pose.pose.position.z;
        
        // 提取姿态四元数并转换为欧拉角
        tf2::Quaternion q(
            _msg->pose.pose.orientation.x,
            _msg->pose.pose.orientation.y,
            _msg->pose.pose.orientation.z,
            _msg->pose.pose.orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        
        ROS_INFO("Received pose: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                x, y, z, roll, pitch, yaw);
        
        // 准备服务请求
        fastlio::SlamReLoc srv_msgs;
        srv_msgs.request.pcd_path = map_path_;
        srv_msgs.request.x = x;
        srv_msgs.request.y = y;
        srv_msgs.request.z = z;
        srv_msgs.request.roll = roll;
        srv_msgs.request.pitch = pitch;
        srv_msgs.request.yaw = yaw;
        
        // 调用重定位服务
        if (reloc_client_.call(srv_msgs)) 
        {
            ROS_INFO("Relocalization service call successful!");
            ROS_INFO("Return status: %d, message: %s", 
                    srv_msgs.response.status, 
                    srv_msgs.response.message.c_str());
        } else 
        {
            ROS_ERROR("Relocalization service call failed!");
        }
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "init_pose");
    init_pose client;
    ros::spin();
    return 0;
}