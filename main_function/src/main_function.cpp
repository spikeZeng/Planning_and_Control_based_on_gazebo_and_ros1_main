#include "main_function.h"

class MyPlanningANDControl : public ros::NodeHandle
{
public:
    MyPlanningANDControl()
    {
        ros::NodeHandle nh_;
        //主车名字
        // role_name = "ego_vehicle";

        //控制器时钟，按照控制时间步来执行控制
        _control_time_step = 0.02;
        // _control_timer = this->create_wall_timer(std::chrono::milliseconds(int(_control_time_step*1000)),std::bind(&MyPlanningANDControl::control_run_step,this));
        // 创建定时器
        _control_timer = std::make_shared<ros::Timer>(nh_.createTimer(ros::Duration(_control_time_step),std::bind(&MyPlanningANDControl::control_run_step,this)));
        _lateral_lqr_controller = std::make_shared<LateralLQRController>();

        //订阅方
        //创建里程计订阅方，订阅车辆当前位置消息
        std::string _odometry_topic_name = "/smart/center_pose";
        _odometry_subscriber = nh_.subscribe(
            _odometry_topic_name, 
            10, 
            &MyPlanningANDControl::odometry_cb, this
        );
        

        //创建惯性导航订阅方，订阅车辆当前速度和角速度消息
        std::string _imu_topic_name = "/smart/velocity";
        _imu_subscriber = nh_.subscribe(
            _imu_topic_name,
            10,
            &MyPlanningANDControl::imu_cb,this
        );
        

        //创建路径订阅方
        _global_path = std::make_shared<std::vector<PathPoint>>();
        std::string _path_subscriber_topic_name = "/final_waypoints";
        _path_subscriber = nh_.subscribe(
            _path_subscriber_topic_name,
            10,
            &MyPlanningANDControl::path_cb,this
        );

        //发布方
        std::string _control_cmd_publisher_topic_name = "/smart/cmd_vel";
        _control_cmd_publisher = nh_.advertise<geometry_msgs::Twist>(
            _control_cmd_publisher_topic_name,
            10
        );

        _current_ego_state = std::make_shared<VehicleState>();
        _current_ego_state->flag_imu = false;
        _current_ego_state->flag_ode = false;
        _current_ego_state->flag_info = true;

    }

private:

        //参数
        // double MIN_DISTANCE_PERCENTAGE = ;
        double _control_time_step;

        //订阅方
        ros::NodeHandle nh_;
        ros::Subscriber _odometry_subscriber; //里程计订阅方，订阅本车当前位姿与速度
        ros::Subscriber _imu_subscriber; //惯性导航订阅方，订阅加速度与角速度
        // ros::Subscriber _ego_info_subscriber; //订阅车辆车道信息
        ros::Subscriber _path_subscriber; //创建路径订阅方

        std::shared_ptr<VehicleState> _current_ego_state;
        // double _reference_speed;//期望速度,单位为km/h
        std::shared_ptr<std::vector<PathPoint>> _global_path;//全局路径存储器

        //发布方
        ros::Publisher _control_cmd_publisher; //控制指令发布

        //控制环节
        std::shared_ptr<ros::Timer> _control_timer;
        std::shared_ptr<LateralLQRController> _lateral_lqr_controller;//横向LQR控制器
        

public:
    //毁掉函数
    //里程计订阅方的回调函数，获取当前本车位姿与速度
    void odometry_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        _current_ego_state->x = msg->pose.position.x;
        _current_ego_state->y = msg->pose.position.y;
        _current_ego_state->z = msg->pose.position.z;
                    
        
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->pose.orientation, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        _current_ego_state->heading = tf2NormalizeAngle(yaw);
        _current_ego_state->flag_ode = true;
        // 里程计的单位是m/s，转化为km/h
    }

    //速度与角速度订阅
    void imu_cb(const geometry_msgs::TwistStamped::ConstPtr& imu_msg)
    {
        // if(imu_msg->linear_acceleration.x >= 10 || imu_msg->linear_acceleration.y >= 10)
        // {
        //     return;
        // }
        // _current_ego_state->ax = imu_msg->linear_acceleration.x;
        // _current_ego_state->ay = imu_msg->linear_acceleration.y;
        _current_ego_state->v = std::sqrt(std::pow(imu_msg->twist.linear.x, 2)
                                       + std::pow(imu_msg->twist.linear.y, 2)
                                       + std::pow(imu_msg->twist.linear.z, 2));
        _current_ego_state->omega = imu_msg->twist.angular.z;
        _current_ego_state->flag_imu = true;

    }


    //全局路径
    void path_cb(const styx_msgs::Lane::ConstPtr& currentWaypoints)
    {
        ROS_INFO("接收到全局路径信息......");
        for(auto &&point : currentWaypoints->waypoints)
        {
            PathPoint temp_path_point;
            temp_path_point.x = point.pose.pose.position.x;
            temp_path_point.y = point.pose.pose.position.y;
            _global_path->push_back(temp_path_point);
        }
        Calculate_heading_and_kappa(_global_path);
    }

    void control_run_step()
    {
        if(_current_ego_state->flag_imu == false || _current_ego_state->flag_ode == false || _current_ego_state->flag_info == false)
        {
            ROS_INFO("等待主车信息......");
            return;
        }

        //由轨迹搜索目标点
        TrajectoryPoint target_point;
        // double cur_time = this->now().seconds();
        // double predicted_time = cur_time + 0.2;
        // int target_point_index = -1;

        //预测模块
        TrajectoryPoint predict_point; //预测0.2s后车辆的位置
        double ts = 0.1; //预测时间
        predict_point.x = _current_ego_state->x + cos(_current_ego_state->heading)*ts*_current_ego_state->v;
        predict_point.y = _current_ego_state->y + sin(_current_ego_state->heading)*ts*_current_ego_state->v;
        predict_point.heading = _current_ego_state->heading +ts*_current_ego_state->omega;
        // predict_point.time_stamped = predicted_time;

        //寻找匹配点
        int match_point_index = 0;

        PathPoint match_point;
        PathPoint projection_point;
        Calculate_projection_point(_global_path, predict_point, match_point_index, match_point, projection_point);

        target_point.x = projection_point.x;
        target_point.y = projection_point.y;
        target_point.heading = projection_point.heading;
        target_point.kappa = projection_point.kappa;

        // double s_dot = _current_ego_state->v*cos(_current_ego_state->heading - target_point.heading)/(1 - target_point.kappa*ed);
        target_point.v = 2;
        // target_point.time_stamped = cur_time;

        //当前车辆位置与目标点距离
        double l = std::sqrt(std::pow(_current_ego_state->x - target_point.x, 2) + std::pow(_current_ego_state->y - target_point.y, 2));

        // #ifdef MAIN_DEBUG
        // RCLCPP_INFO(this->get_logger(),"当前位姿(%.3f,%.3f,%.3f,%.3f),期望位姿(%.3f,%.3f,%.3f,%.3f)",
        //             _current_ego_state->x,_current_ego_state->y,_current_ego_state->heading,_current_ego_state->v,
        //             target_point.x,target_point.y,target_point.heading, target_point.v);
        // // RCLCPP_INFO(this->get_logger(),"参考线起点位姿(%.3f,%.3f,%.3f),参考线终点位姿(%.3f,%.3f,%.3f)",
        // //             _reference_line->at(0).x,_reference_line->at(0).y,_reference_line->at(0).heading,
        // //             _reference_line->back().x,_reference_line->back().y,_reference_line->back().heading);

        // #endif
        

        //控制指令
        geometry_msgs::Twist twistCmd;
        // if(l > 0.5)
        // {
        //     twistCmd.linear.x = 2.0;
        //     twistCmd.angular.z = _lateral_lqr_controller->run_step(target_point,*_current_ego_state,_control_time_step);
        // }
        // else
        // {
        //     twistCmd.linear.x = 2.0;
        //     twistCmd.angular.z = 0.0;
        // }

        twistCmd.linear.x = 2.0;
        twistCmd.angular.z = _lateral_lqr_controller->run_step(target_point,*_current_ego_state,_control_time_step);

        _control_cmd_publisher.publish(twistCmd);

    }

    
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MyPlanningANDControl");

    setlocale(LC_ALL, "");

    std::shared_ptr<MyPlanningANDControl>  node = std::make_shared<MyPlanningANDControl>();

    ros::spin();

    return 0;
}