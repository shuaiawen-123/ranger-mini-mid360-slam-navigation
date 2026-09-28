#include <ros/ros.h>
#include "fastlio/SaveMap.h"

int main(int argc, char** argv) 
{
    // Initialize ROS node
    ros::init(argc, argv, "save_map_client");
    
    // Create node handle
    ros::NodeHandle nh,nh_p("~");

    std::string save_path;
    nh_p.param<std::string>("save_map_path", save_path, "");
    
    // Create service client
    ros::ServiceClient client = nh.serviceClient<fastlio::SaveMap>("save_map");
    
    // Wait for service to become available
    ROS_INFO("Waiting for save_map service...");
    if (!client.waitForExistence(ros::Duration(5.0))) 
    {
        ROS_ERROR("Service save_map not available after 5 seconds");
    }
    ROS_INFO("Service connected!");
    
    // Create service request/response objects
    fastlio::SaveMap srv_msgs;
    
    // Set save path in request
    srv_msgs.request.save_path = save_path;
    srv_msgs.request.resolution = 0.0;
    
    // Call service
    ROS_INFO("Requesting map save to: %s", save_path.c_str());
    if (client.call(srv_msgs)) 
    {
        if (srv_msgs.response.status) 
        {
            ROS_INFO("Map saved successfully!");
            ROS_INFO("Service message: %s", srv_msgs.response.message.c_str());
        } else 
        {
            ROS_ERROR("Map save failed!");
            ROS_ERROR("Error message: %s", srv_msgs.response.message.c_str());
        }
    } 
    else 
    {
        ROS_ERROR("Failed to call save_map service");
    }
    
    return 0;
}