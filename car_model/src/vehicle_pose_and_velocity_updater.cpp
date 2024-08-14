#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// 定义类 VehiclePoseAndVelocityUpdater
class VehiclePoseAndVelocityUpdater
{
public:
    // 构造函数
    VehiclePoseAndVelocityUpdater()
    {
        ros::NodeHandle nh;

        // 初始化发布者，发布车辆后轮和中心的位置信息，以及速度信息
        rear_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/smart/rear_pose", 1);
        center_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/smart/center_pose", 1);
        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/smart/velocity", 1);

        // 初始化订阅者，订阅 Gazebo 模型状态
        model_sub_ = nh.subscribe("/gazebo/model_states", 1, &VehiclePoseAndVelocityUpdater::modelCallback, this);

        ros::spin(); // 保持节点运行
    }

private:
    // 回调函数，当接收到模型状态信息时被调用
    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& data)
    {
        // 查找模型名为 "smart" 的索引
        auto it = std::find(data->name.begin(), data->name.end(), "smart");
        if (it == data->name.end()) return; // 如果找不到，直接返回
        int vehicle_model_index = std::distance(data->name.begin(), it);

        // 获取车辆位置和速度
        geometry_msgs::Pose vehicle_position = data->pose[vehicle_model_index];
        geometry_msgs::Twist vehicle_velocity = data->twist[vehicle_model_index];

        // 将四元数转换为欧拉角（滚转、俯仰、偏航）
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(vehicle_position.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        ros::Time time_stamp = ros::Time::now(); // 获取当前时间戳

        // 发布车辆中心位置
        geometry_msgs::PoseStamped center_pose;
        center_pose.header.frame_id = "/world";
        center_pose.header.stamp = time_stamp;
        center_pose.pose.position = vehicle_position.position;
        center_pose.pose.orientation = vehicle_position.orientation;
        center_pose_pub_.publish(center_pose);

        // 计算并发布车辆后轮轴位置
        geometry_msgs::PoseStamped rear_pose;
        rear_pose.header.frame_id = "/world";
        rear_pose.header.stamp = time_stamp;
        double center_x = vehicle_position.position.x;
        double center_y = vehicle_position.position.y;
        double rear_x = center_x - cos(yaw) * 0.945; // 根据车辆中心位置和偏航角计算后轮位置
        double rear_y = center_y - sin(yaw) * 0.945;
        rear_pose.pose.position.x = rear_x;
        rear_pose.pose.position.y = rear_y;
        rear_pose.pose.orientation = vehicle_position.orientation;
        rear_pose_pub_.publish(rear_pose);

        // 发布车辆速度
        geometry_msgs::TwistStamped velocity;
        velocity.header.frame_id = "";
        velocity.header.stamp = time_stamp;
        velocity.twist.linear = vehicle_velocity.linear;
        velocity.twist.angular = vehicle_velocity.angular;
        vel_pub_.publish(velocity);
    }

    // 声明订阅者和发布者
    ros::Subscriber model_sub_;
    ros::Publisher rear_pose_pub_;
    ros::Publisher center_pose_pub_;
    ros::Publisher vel_pub_;
};

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_pose_and_velocity_updater"); // 初始化 ROS 节点
    try
    {
        VehiclePoseAndVelocityUpdater(); // 创建 VehiclePoseAndVelocityUpdater 对象
    }
    catch (const ros::Exception& e)
    {
        ROS_WARN("Cannot start vehicle pose and velocity updater: %s", e.what()); // 捕获异常并打印警告信息
    }
    return 0;
}
