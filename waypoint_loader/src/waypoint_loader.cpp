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

#define MAX_DECEL 1.0

class WaypointLoader {
public:
    WaypointLoader() {
        // 初始化ROS节点
        ros::NodeHandle nh;
        ROS_INFO("Initializing WaypointLoader node...");

        // 获取参数
        velocity_ = 10;
        path_ = "/home/spikesong/Planning_and_Control_based_on_gazebo_and_ros1_main/src/waypoint_loader/waypoints/waypoints.csv";
        ROS_INFO("Velocity: %f, Path: %s", velocity_, path_.c_str());

        // 设置发布者
        pub_ = nh.advertise<styx_msgs::Lane>("/base_waypoints", 1, true);
        pub_path_ = nh.advertise<nav_msgs::Path>("/base_path", 1, true);

        // 将速度从 km/h 转换为 m/s
        velocity_ = kmph2mps(velocity_);

        // 加载路径点
        newWaypointLoader(path_);

        ros::spin();
    }

private:
    ros::Publisher pub_;
    ros::Publisher pub_path_;
    double velocity_;
    std::string path_;

    // 将速度从 km/h 转换为 m/s
    double kmph2mps(double velocity_kmph) {
        return (velocity_kmph * 1000.) / (60. * 60.);
    }

    // 加载新的路径点
    void newWaypointLoader(const std::string& path) {
        std::vector<styx_msgs::Waypoint> waypoints;
        nav_msgs::Path base_path;
        ROS_INFO("Loading waypoints from path: %s", path.c_str());
        if (loadWaypoints(path, waypoints, base_path)) {
            publish(waypoints, base_path);
            ROS_INFO("Waypoints Loaded");
        } else {
            ROS_ERROR("%s is not a file", path.c_str());
        }
    }

    // 从文件加载路径点
    bool loadWaypoints(const std::string& filename, std::vector<styx_msgs::Waypoint>& waypoints, nav_msgs::Path& base_path) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Unable to open file: %s", filename.c_str());
            return false;
        }
        ROS_INFO("File opened successfully: %s", filename.c_str());

        base_path.header.frame_id = "world";

        std::string line;
        while (std::getline(file, line)) {
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
            waypoint.twist.twist.linear.x = velocity_;
            waypoint.forward = true;
            waypoints.push_back(waypoint);

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = waypoint.pose.pose.position.x;
            pose.pose.position.y = waypoint.pose.pose.position.y;
            base_path.poses.push_back(pose);
        }
        ROS_INFO("Loaded %lu waypoints", waypoints.size());

        decelerate(waypoints);
        return true;
    }

    // 计算两个点之间的距离
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        double x = p1.x - p2.x;
        double y = p1.y - p2.y;
        double z = p1.z - p2.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    // 减速
    void decelerate(std::vector<styx_msgs::Waypoint>& waypoints) {
        if (waypoints.empty()) return;

        auto& last = waypoints.back();
        last.twist.twist.linear.x = 0.0;

        for (auto it = waypoints.rbegin() + 1; it != waypoints.rend(); ++it) {
            double dist = distance(it->pose.pose.position, last.pose.pose.position);
            double vel = std::sqrt(2 * MAX_DECEL * dist);
            if (vel < 1.0) {
                vel = 0.0;
            }
            it->twist.twist.linear.x = std::min(vel, it->twist.twist.linear.x);
        }
        ROS_INFO("Deceleration complete.");
    }

    // 发布路径点
    void publish(const std::vector<styx_msgs::Waypoint>& waypoints, const nav_msgs::Path& base_path) {
        styx_msgs::Lane lane;
        lane.header.frame_id = "world";
        lane.header.stamp = ros::Time(0);
        lane.waypoints = waypoints;

        pub_.publish(lane);
        pub_path_.publish(base_path);    

    }
};

int main(int argc, char* argv[]) {
    // 初始化ROS节点
    ros::init(argc, argv, "waypoint_loader");
    try {
        // 创建WaypointLoader对象，启动节点
        WaypointLoader wp;
    } catch (ros::Exception& e) {
        ROS_ERROR("Could not start waypoint node: %s", e.what());
    }

    return 0;
}
