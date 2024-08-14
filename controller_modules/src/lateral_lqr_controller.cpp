#include "lateral_lqr_controller.h"

LateralLQRController::LateralLQRController()
{
    //车辆静态参数初始化
    _cf = -155494.663;
    _cr = -155494.663;
    _m = 1845.0;
    _a = 2.852/2.0;
    _b = 2.852/2.0;
    _Iz = _a*_a*(_m/2.0) + _b*_b*(_m/2.0);
    _steer_ratio = 16.0;

    //lqr静态矩阵
    _matrix_size = 4;
    _matrix_A  = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_A_bar = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_B = Eigen::MatrixXd::Zero(_matrix_size, 1);
    _matrix_B_bar = Eigen::MatrixXd::Zero(_matrix_size, 1);
    _matrix_K = Eigen::MatrixXd::Zero(1, _matrix_size);
    _matrix_err = Eigen::VectorXd::Zero(_matrix_size);
    _matrix_Q = Eigen::MatrixXd::Identity(_matrix_size, _matrix_size);
    _matrix_R = Eigen::MatrixXd::Identity(1, 1);

    _matrix_B(1, 0) = -_cf/_m;
    _matrix_B(3, 0) = -_a*_cf/_Iz;

    _matrix_R(0, 0) = 2; //输入权重
    
    _matrix_Q(0, 0) = 5; //横向距离误差权重
    _matrix_Q(1, 1) = 1.5; //横向距离误差导数权重
    _matrix_Q(2, 2) = 10; //横向角度误差权重
    _matrix_Q(3, 3) = 5; //横向角度误差导数权重

    _iter_max = 1500;
    _tolerance = 0.01;
}

double LateralLQRController::run_step(const TrajectoryPoint& match_point, const VehicleState& current_ego_state,
                                      const double& control_time_step)
{
    ROS_INFO("laterl_lqr_controller");
    //1.创建状态矩阵
    _vx = current_ego_state.v + 1e-4;
    _matrix_A(0, 1) = 1.0;
    _matrix_A(1,1) = (_cf + _cr)/(_m * _vx);
    _matrix_A(1,2) = -(_cf + _cr)/(_m);
    _matrix_A(1,3) = (_a*_cf - _b*_cr)/(_m * _vx);
    _matrix_A(2,3) = 1.0;
    _matrix_A(3,1) = (_a*_cf - _b*_cr)/(_Iz * _vx);
    _matrix_A(3,2) = -(_a*_cf - _b*_cr)/(_Iz);
    _matrix_A(3,3) = (_a*_a*_cf + _b*_b*_cr)/(_Iz * _vx);

    //2.离散化，前向欧拉离散
    auto I = Eigen::MatrixXd::Identity(_matrix_size, _matrix_size);
    _matrix_A_bar = (I + _matrix_A*control_time_step);
    _matrix_B_bar = _matrix_B * control_time_step;

    //3.求解反馈矩阵
    if(! SolveLQRFeedback(_matrix_A_bar, _matrix_B_bar, _matrix_Q, _matrix_R,_iter_max,_tolerance))
    {
        ROS_INFO("反馈矩阵K求解失败");
        _matrix_K = Eigen::MatrixXd::Zero(1,_matrix_size);
    }

    //4.计算误差
    Eigen::Vector2d tao(std::cos(match_point.heading),std::sin(match_point.heading));
    Eigen::Vector2d nor(-std::sin(match_point.heading),std::cos(match_point.heading));
    Eigen::Vector2d vec_d(current_ego_state.x - match_point.x, current_ego_state.y - match_point.y);

    double ed = nor.transpose()*vec_d; //横向误差
    // double es = tao.transpose() * vec_d;//纵向距离误差

    //double theta_r = match_point.heading;//Apollo版本
    //double theta_r = match_point.heading + match_point.kappa * es;//老王版本
    double ephi = current_ego_state.heading - match_point.heading;
    //修正角度误差
    if(ephi > M_PI)
    {
        ephi = ephi - 2*M_PI;
    }
    else if(ephi < -M_PI)
    {
        ephi = ephi + 2*M_PI;
    }
    ephi = std::sin(ephi);

    double ed_dot= current_ego_state.v * std::sin(ephi);
    // double ephi_dot = current_ego_state.omega - match_point.kappa*match_point.v;
    
    double s_dot = current_ego_state.v*cos(current_ego_state.heading - match_point.heading)/(1 - match_point.kappa*ed);
    double ephi_dot = current_ego_state.omega - match_point.kappa*s_dot;

    _matrix_err[0] = ed;
    _matrix_err[1] = ed_dot;
    _matrix_err[2] = ephi;
    _matrix_err[3] = ephi_dot;

    //5.计算前馈
    double k3 = _matrix_K(0, 2);
    double k_m = match_point.kappa;
    double delta_f = k_m*(_a + _b - _b*k3 - (_m*_vx*_vx/(_a + _b))*(_b/_cf + _a*k3/_cr - _a/_cr));

    //6.计算前轮转角
    double u = -(_matrix_K*_matrix_err)[0] + delta_f;
    //限制前轮转角
    double max_u = 20.0 * M_PI / 180.0;
    u = std::min(std::max(u, -max_u), max_u);

    //7.计算方向盘转角
    double steer = -u; //前轮转角计算出的负值为右转，

    return std::min(std::max(steer, -1.0), 1.0);
}

bool LateralLQRController::SolveLQRFeedback(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, 
                            const Eigen::MatrixXd& R, const int& iter_max, const double& _tolerance)
{
    ROS_INFO("laterl_lqr_controller");
    //判断输入矩阵的维数是否正确
    if(A.rows() != A.cols() || B.rows() != A.cols() || Q.rows()!=Q.cols() || Q.rows()!=A.rows() || R.cols()!=B.cols())
    {
        ROS_ERROR("输入矩阵维数不匹配");
        return false;
    }

    auto matrix_size = A.rows();
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd AT;
    Eigen::MatrixXd BT;

    //初始化
    P = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    P_next = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    AT = A.transpose();
    BT = B.transpose();

    //迭代计算黎卡提方程
    for(int i = 0; i < iter_max; i++){
        P_next = Q + AT*P*A - AT*P*B*(R + BT*P*B).inverse()*BT*P*A;
        if(fabs((P_next - P).maxCoeff()) < _tolerance){
            _matrix_K = (R + BT*P*B).inverse()*(BT*P*A);
            return true;
        }
        //迭代P
        P = P_next;
    }
    return false;
}                   