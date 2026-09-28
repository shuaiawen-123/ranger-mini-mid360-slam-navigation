#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <algorithm>
#include <string>

class CmdVelMux
{
public:
    CmdVelMux()
        : nh_(),
          pnh_("~"),
          have_nav_(false),
          have_turn_(false)
    {
        pnh_.param<std::string>(
            "nav_topic", nav_topic_, "/cmd_vel_nav");

        pnh_.param<std::string>(
            "turn_topic", turn_topic_, "/cmd_vel_turn");

        pnh_.param<std::string>(
            "output_topic", output_topic_, "/cmd_vel");

        pnh_.param("nav_timeout", nav_timeout_, 0.5);
        pnh_.param("turn_timeout", turn_timeout_, 0.25);
        pnh_.param("publish_rate", publish_rate_, 30.0);

        output_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                output_topic_, 10);

        nav_sub_ = nh_.subscribe(
            nav_topic_,
            10,
            &CmdVelMux::navCallback,
            this,
            ros::TransportHints().tcpNoDelay());

        turn_sub_ = nh_.subscribe(
            turn_topic_,
            10,
            &CmdVelMux::turnCallback,
            this,
            ros::TransportHints().tcpNoDelay());

        timer_ = nh_.createWallTimer(
            ros::WallDuration(
                1.0 / std::max(1.0, publish_rate_)),
            &CmdVelMux::timerCallback,
            this);

        ROS_INFO_STREAM(
            "cmd_vel mux started"
            << "\n  navigation input: " << nav_topic_
            << "\n  turn input: " << turn_topic_
            << "\n  output: " << output_topic_);
    }

    ~CmdVelMux()
    {
        geometry_msgs::Twist zero;

        ros::WallRate rate(20.0);

        for (int i = 0; i < 20; ++i)
        {
            output_pub_.publish(zero);
            rate.sleep();
        }
    }

private:
    void navCallback(
        const geometry_msgs::Twist::ConstPtr& message)
    {
        nav_command_ = *message;
        nav_time_ = ros::WallTime::now();
        have_nav_ = true;
    }

    void turnCallback(
        const geometry_msgs::Twist::ConstPtr& message)
    {
        turn_command_ = *message;
        turn_time_ = ros::WallTime::now();
        have_turn_ = true;
    }

    void timerCallback(const ros::WallTimerEvent&)
    {
        const ros::WallTime now = ros::WallTime::now();

        const bool turn_fresh =
            have_turn_ &&
            (now - turn_time_).toSec() <= turn_timeout_;

        const bool nav_fresh =
            have_nav_ &&
            (now - nav_time_).toSec() <= nav_timeout_;

        geometry_msgs::Twist output;

        // 掉头指令优先级最高
        if (turn_fresh)
        {
            output = turn_command_;
        }
        else if (nav_fresh)
        {
            output = nav_command_;
        }

        output_pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber nav_sub_;
    ros::Subscriber turn_sub_;
    ros::Publisher output_pub_;
    ros::WallTimer timer_;

    geometry_msgs::Twist nav_command_;
    geometry_msgs::Twist turn_command_;

    ros::WallTime nav_time_;
    ros::WallTime turn_time_;

    bool have_nav_;
    bool have_turn_;

    std::string nav_topic_;
    std::string turn_topic_;
    std::string output_topic_;

    double nav_timeout_;
    double turn_timeout_;
    double publish_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "round_trip_cmd_vel_mux");

    CmdVelMux mux;
    ros::spin();

    return 0;
}
