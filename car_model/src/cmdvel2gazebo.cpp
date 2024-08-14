#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// 定义类 CmdVel2Gazebo
class CmdVel2Gazebo
{
public:
    // 构造函数
    CmdVel2Gazebo()
    {
        ros::NodeHandle nh;

        // 初始化订阅者，订阅 /smart/cmd_vel 主题的消息
        sub_ = nh.subscribe("/smart/cmd_vel", 1, &CmdVel2Gazebo::callback, this);

        // 初始化发布者，发布转向和后轮速度的命令
        pub_steerL_ = nh.advertise<std_msgs::Float64>("/smart/front_left_steering_position_controller/command", 1);
        pub_steerR_ = nh.advertise<std_msgs::Float64>("/smart/front_right_steering_position_controller/command", 1);
        pub_rearL_ = nh.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 1);
        pub_rearR_ = nh.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 1);

        x_ = 0; // 初始线速度为0
        z_ = 0; // 初始角速度为0

        L_ = 1.868; // 车辆轴距
        T_front_ = 1.284; // 前轮距
        T_rear_ = 1.284; // 后轮距

        timeout_ = ros::Duration(0.2); // 超时时间设为0.2秒
        lastMsg_ = ros::Time::now(); // 记录当前时间

        maxsteerInside_ = 0.6; // 内侧车轮最大转向角

        // 计算内侧车轮最大转向角对应的最小转弯半径
        double rMax = L_ / std::tan(maxsteerInside_);
        // 计算理想中间车轮的转弯半径
        double rIdeal = rMax + (T_front_ / 2.0);
        // 计算理想中间车轮的最大转向角
        maxsteer_ = std::atan2(L_, rIdeal);

        ros::Rate rate(10); // 设置循环频率为10Hz
        while (ros::ok())
        {
            publish(); // 发布转向和速度命令
            ros::spinOnce(); // 处理所有回调函数
            rate.sleep(); // 睡眠以保持循环频率
        }
    }

private:
    // 回调函数，当接收到 /smart/cmd_vel 主题的消息时被调用
    void callback(const geometry_msgs::Twist::ConstPtr& data)
    {
        x_ = data->linear.x / 0.3; // 更新线速度
        z_ = std::max(-maxsteer_, std::min(maxsteer_, data->angular.z)); // 更新角速度，并限制其在最大转向角范围内
        lastMsg_ = ros::Time::now(); // 更新最后接收到消息的时间
    }

    // 发布转向和速度命令
    void publish()
    {
        ros::Duration delta_last_msg_time = ros::Time::now() - lastMsg_; // 计算距离最后接收到消息的时间
        bool msgs_too_old = delta_last_msg_time > timeout_; // 判断消息是否超时
        if (msgs_too_old)
        {
            x_ = 0; // 如果超时，则将线速度设为0
            std_msgs::Float64 msgRear;
            msgRear.data = x_;
            pub_rearL_.publish(msgRear);
            pub_rearR_.publish(msgRear);
            std_msgs::Float64 msgSteer;
            msgSteer.data = 0;
            pub_steerL_.publish(msgSteer);
            pub_steerR_.publish(msgSteer);

            return;
        }

        if (z_ != 0) // 如果角速度不为0
        {
            double r = L_ / std::fabs(std::tan(z_)); // 计算转弯半径

            // 计算各个车轮的转弯半径
            double rL_rear = r - (std::copysign(1.0, z_) * (T_rear_ / 2.0));
            double rR_rear = r + (std::copysign(1.0, z_) * (T_rear_ / 2.0));
            double rL_front = r - (std::copysign(1.0, z_) * (T_front_ / 2.0));
            double rR_front = r + (std::copysign(1.0, z_) * (T_front_ / 2.0));

            // 发布后轮速度
            std_msgs::Float64 msgRearR;
            msgRearR.data = x_ * rR_rear / r;
            std_msgs::Float64 msgRearL;
            msgRearL.data = x_ * rL_rear / r;

            pub_rearL_.publish(msgRearL);
            pub_rearR_.publish(msgRearR);

            // 发布前轮转向角
            std_msgs::Float64 msgSteerL;
            std_msgs::Float64 msgSteerR;
            msgSteerL.data = std::atan2(L_, rL_front) * std::copysign(1.0, z_);
            pub_steerL_.publish(msgSteerL);
            msgSteerR.data = std::atan2(L_, rR_front) * std::copysign(1.0, z_);
            pub_steerR_.publish(msgSteerR);
        }
        else // 如果角速度为0
        {
            // 发布后轮速度
            std_msgs::Float64 msgRear;
            msgRear.data = x_;
            pub_rearL_.publish(msgRear);
            pub_rearR_.publish(msgRear);

            // 发布前轮转向角
            std_msgs::Float64 msgSteer;
            msgSteer.data = z_;
            pub_steerL_.publish(msgSteer);
            pub_steerR_.publish(msgSteer);
        }
    }

    // 声明订阅者和发布者
    ros::Subscriber sub_;
    ros::Publisher pub_steerL_;
    ros::Publisher pub_steerR_;
    ros::Publisher pub_rearL_;
    ros::Publisher pub_rearR_;

    // 定义私有成员变量
    double x_; // 线速度
    double z_; // 角速度

    double L_; // 车辆轴距
    double T_front_; // 前轮距
    double T_rear_; // 后轮距

    ros::Duration timeout_; // 超时时间
    ros::Time lastMsg_; // 最后接收到消息的时间

    double maxsteerInside_; // 内侧车轮最大转向角
    double maxsteer_; // 理想中间车轮的最大转向角
};

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmdvel2gazebo"); // 初始化 ROS 节点
    try
    {
        CmdVel2Gazebo(); // 创建 CmdVel2Gazebo 对象
    }
    catch (const ros::Exception& e)
    {
        ROS_WARN("Cannot start vehicle pose and velocity updater: %s", e.what()); // 捕获异常并打印警告信息
    }
    return 0;
}
