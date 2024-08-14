#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <styx_msgs/Lane.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

#define HORIZON 6.0

class PurePersuit {
public:
    PurePersuit() {
        ros::NodeHandle nh;

        // 初始化订阅者
        pose_sub_ = nh.subscribe("/smart/rear_pose", 1, &PurePersuit::pose_cb, this);
        velocity_sub_ = nh.subscribe("/smart/velocity", 1, &PurePersuit::vel_cb, this);
        waypoints_sub_ = nh.subscribe("/final_waypoints", 1, &PurePersuit::lane_cb, this);

        // 初始化发布者
        twist_pub_ = nh.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 1);

        // 初始化成员变量
        currentPose_ = nullptr;
        currentVelocity_ = nullptr;
        currentWaypoints_ = nullptr;

        // 循环
        loop();
    }

private:
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Publisher twist_pub_;

    geometry_msgs::PoseStamped::ConstPtr currentPose_;
    geometry_msgs::TwistStamped::ConstPtr currentVelocity_;
    styx_msgs::Lane::ConstPtr currentWaypoints_;

    void loop() {
        ros::Rate rate(20);
        ROS_WARN("pure persuit starts");
        while (ros::ok()) {
            if (currentPose_ && currentVelocity_ && currentWaypoints_) {
                geometry_msgs::Twist twistCommand = calculateTwistCommand();
                twist_pub_.publish(twistCommand);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& data) {
        currentPose_ = data;
    }

    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& data) {
        currentVelocity_ = data;
    }

    void lane_cb(const styx_msgs::Lane::ConstPtr& data) {
        currentWaypoints_ = data;
    }

    geometry_msgs::Twist calculateTwistCommand() {
        double lad = 0.0; // look ahead distance accumulator
        size_t targetIndex = currentWaypoints_->waypoints.size() - 1;
        for (size_t i = 0; i < currentWaypoints_->waypoints.size(); ++i) {
            if ((i + 1) < currentWaypoints_->waypoints.size()) {
                double this_x = currentWaypoints_->waypoints[i].pose.pose.position.x;
                double this_y = currentWaypoints_->waypoints[i].pose.pose.position.y;
                double next_x = currentWaypoints_->waypoints[i + 1].pose.pose.position.x;
                double next_y = currentWaypoints_->waypoints[i + 1].pose.pose.position.y;
                lad += std::hypot(next_x - this_x, next_y - this_y);
                if (lad > HORIZON) {
                    targetIndex = i + 1;
                    break;
                }
            }
        }

        const auto& targetWaypoint = currentWaypoints_->waypoints[targetIndex];
        double targetSpeed = currentWaypoints_->waypoints[0].twist.twist.linear.x;

        double targetX = targetWaypoint.pose.pose.position.x;
        double targetY = targetWaypoint.pose.pose.position.y;
        double currentX = currentPose_->pose.position.x;
        double currentY = currentPose_->pose.position.y;

        // 获取车辆的偏航角
        tf::Quaternion q(
            currentPose_->pose.orientation.x,
            currentPose_->pose.orientation.y,
            currentPose_->pose.orientation.z,
            currentPose_->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 获取角度差异
        double alpha = std::atan2(targetY - currentY, targetX - currentX) - yaw;
        double l = std::sqrt(std::pow(currentX - targetX, 2) + std::pow(currentY - targetY, 2));

        geometry_msgs::Twist twistCmd;
        if (l > 0.5) {
            double theta = std::atan(2 * 1.868 * std::sin(alpha) / l);
            // 获取twist命令
            twistCmd.linear.x = targetSpeed;
            twistCmd.angular.z = theta;
        } else {
            twistCmd.linear.x = 0;
            twistCmd.angular.z = 0;
        }

        return twistCmd;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_persuit");
    try {
        PurePersuit pp;
    } catch (const ros::Exception& e) {
        ROS_ERROR("Could not start motion control node: %s", e.what());
    }
    return 0;
}
