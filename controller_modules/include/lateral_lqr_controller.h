#pragma once 

#include <Eigen/Dense>
#include <ros/ros.h>
#include <common.h>

class LateralLQRController
{
public:
    LateralLQRController();

private:
    //车辆参数
    double _cf; //前轮侧偏刚度系数
    double _cr; //后轮侧偏刚度系数
    double _m;  //汽车质量
    double _vx; //汽车轴线上速度
    double _Iz; //转动惯量
    double _a; //质心到前轴距离
    double _b; //质心到后轴距离
    double _steer_ratio; //方向盘转角到前轮转向角的传动比

    //lqr参数
    int _matrix_size;
    Eigen::MatrixXd _matrix_A; //状态矩阵
    Eigen::MatrixXd _matrix_A_bar; //离散化状态矩阵
    Eigen::MatrixXd _matrix_B; //输入矩阵
    Eigen::MatrixXd _matrix_B_bar; //离散化矩阵
    Eigen::MatrixXd _matrix_K; //反馈矩阵
    Eigen::VectorXd _matrix_err; //误差向量
    Eigen::MatrixXd _matrix_Q; //Q矩阵
    Eigen::MatrixXd _matrix_R; //R矩阵
    int _iter_max; //最到迭代次数
    double _tolerance; //迭代精度
public:
    bool SolveLQRFeedback(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, 
                            const Eigen::MatrixXd& R, const int& iter_max, const double& _tolerance);
    double run_step(const TrajectoryPoint& match_point, const VehicleState& current_ego_state,
                    const double& control_time_step);

};