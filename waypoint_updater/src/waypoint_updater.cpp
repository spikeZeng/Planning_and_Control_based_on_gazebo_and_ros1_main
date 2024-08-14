#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <styx_msgs/Lane.h>
#include <styx_msgs/Waypoint.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#define LOOKAHEAD_WPS 20 // Number of waypoints we will publish. You can change this number

class WaypointUpdater {
public:
    WaypointUpdater() {
        ros::NodeHandle nh;

        // Subscribe to topics
        pose_sub_ = nh.subscribe("/smart/rear_pose", 1, &WaypointUpdater::pose_cb, this);
        waypoints_sub_ = nh.subscribe("/base_waypoints", 1, &WaypointUpdater::waypoints_cb, this);

        // Publishers
        final_waypoints_pub_ = nh.advertise<styx_msgs::Lane>("final_waypoints", 1);
        final_path_pub_ = nh.advertise<nav_msgs::Path>("final_path", 1);

        base_waypoints_ = nullptr;
        pose_ = nullptr;

        ros::Rate rate(20);
        while (ros::ok()) {
            if (pose_ && base_waypoints_) {
                // Get closest waypoint
                int closest_waypoint_idx = get_closest_waypoint_idx();
                publish_waypoints(closest_waypoint_idx);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber pose_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Publisher final_waypoints_pub_;
    ros::Publisher final_path_pub_;

    styx_msgs::Lane* base_waypoints_;
    geometry_msgs::PoseStamped* pose_;

    // Callback for pose
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pose_ = new geometry_msgs::PoseStamped(*msg);
    }

    // Callback for base waypoints
    void waypoints_cb(const styx_msgs::Lane::ConstPtr& waypoints) {
        base_waypoints_ = new styx_msgs::Lane(*waypoints);
    }

    // Get the closest waypoint index
    int get_closest_waypoint_idx() {
        if (!pose_ || !base_waypoints_) return -1;

        double x = pose_->pose.position.x;
        double y = pose_->pose.position.y;

        // Initialize minimum distance
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = -1;

        // Find the closest waypoint
        for (size_t i = 0; i < base_waypoints_->waypoints.size(); i++) {
            double dist = distance(base_waypoints_->waypoints[i].pose.pose.position.x, base_waypoints_->waypoints[i].pose.pose.position.y, x, y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    // Publish waypoints
    void publish_waypoints(int closest_idx) {
        if (closest_idx == -1) return;

        // Fill in final waypoints to publish
        styx_msgs::Lane lane;
        lane.header = base_waypoints_->header;
        int end_idx = std::min(static_cast<int>(base_waypoints_->waypoints.size()), closest_idx + LOOKAHEAD_WPS);
        lane.waypoints.insert(lane.waypoints.end(), base_waypoints_->waypoints.begin() + closest_idx, base_waypoints_->waypoints.begin() + end_idx);

        // Fill in path for visualization in Rviz
        nav_msgs::Path path;
        path.header.frame_id = "/world";
        for (const auto& p : lane.waypoints) {
            geometry_msgs::PoseStamped path_element;
            path_element.pose.position.x = p.pose.pose.position.x;
            path_element.pose.position.y = p.pose.pose.position.y;
            path.poses.push_back(path_element);
        }

        final_waypoints_pub_.publish(lane);
        final_path_pub_.publish(path);
    }

    // Calculate Euclidean distance
    double distance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_updater");

    try {
        // Create WaypointUpdater object and start the node
        WaypointUpdater updater;
    } catch (std::runtime_error& e) {
        ROS_ERROR("Could not start waypoint updater node: %s", e.what());
    }

    return 0;
}
