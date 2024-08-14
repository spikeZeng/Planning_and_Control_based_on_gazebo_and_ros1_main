#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class TransformPublisher {
public:
    TransformPublisher() {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 订阅 /smart/center_pose 话题
        sub_ = nh.subscribe("/smart/center_pose", 1, &TransformPublisher::poseCallback, this);

        // 进入 ROS 事件循环
        ros::spin();
    }

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 获取位置信息
        const geometry_msgs::Point& position = msg->pose.position;
        // 获取方向信息
        const geometry_msgs::Quaternion& orientation = msg->pose.orientation;

        // 创建 TransformBroadcaster 对象
        static tf::TransformBroadcaster br;
        // 发送变换
        br.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
                          tf::Vector3(position.x, position.y, position.z)),
            ros::Time::now(), "world", "base_link"));
    }

    ros::Subscriber sub_; // 订阅者对象
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "transform_publisher");

    try {
        // 创建 TransformPublisher 对象，启动节点
        TransformPublisher tp;
    } catch (...) {
        // 捕获所有异常，并记录警告信息
        ROS_WARN("Cannot start transform publisher");
    }

    return 0;
}

