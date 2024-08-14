#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <styx_msgs/Lane.h>
#include <styx_msgs/Waypoint.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>


/*该函数的功能为调试自定义消息包是否正常
    发布自定义消息waypoint[]:
*/
bool loadWaypoints(const std::string& filename, std::vector<styx_msgs::Waypoint>& waypoints)
{
    std::ifstream file(filename);
    if(!file.is_open())
    {
        ROS_ERROR("Unable to open file: %s", filename.c_str());
        return false;
    }
    ROS_INFO("File opened successfully: %s", filename.c_str());

    std::string line;
    while(std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string x_str, y_str, yaw_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, yaw_str, ',');

        styx_msgs::Waypoint waypoint;
        waypoint.pose.pose.position.x = std::stod(x_str);
        waypoint.pose.pose.position.y = std::stod(y_str);
        double yaw = std::stod(yaw_str);
        waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        waypoint.twist.twist.linear.x = 10;
        waypoint.forward = true;
        waypoints.push_back(waypoint);
    }

    ROS_INFO("Loaded %lu waypoints", waypoints.size());
    return true;

}
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"demo_my_styx_msgs");

    ros::NodeHandle nh;

    ros::Publisher pub_Lane = nh.advertise<styx_msgs::Lane>("/my_Lane",1,true);

    std::vector<styx_msgs::Waypoint> waypoints;

    std::string path = "/home/spikesong/Planning_and_Control_based_on_gazebo_and_ros1_main/src/waypoint_loader/waypoints/waypoints.csv";

    if(loadWaypoints(path, waypoints)){
        ROS_INFO("Waypoints Loaded");
    }
    else{
        ROS_ERROR("%s is not a file", path.c_str());
    }

    styx_msgs::Lane lane;
    lane.header.frame_id = "world";
    lane.header.stamp = ros::Time(0);
    lane.waypoints = waypoints;

    ros::Rate rate(1);
    while(ros::ok())
    {
        pub_Lane.publish(lane);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}