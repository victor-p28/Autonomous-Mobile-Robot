#include "ekf.hpp"
#include <cmath>

EKF::EKF() {
    x = Eigen::Vector3d::Zero();
    P = Eigen::Matrix3d::Identity() * 0.1;
    Q = Eigen::Matrix3d::Identity() * 0.01;
    R = Eigen::Matrix2d::Identity() * 0.2;
    H << 1, 0, 0,
         0, 1, 0;
}

void EKF::predict(double v, double omega, double dt) {
    Eigen::Matrix3d F;
    F << 1, 0, -v * dt * sin(x(2)),
         0, 1, v * dt * cos(x(2)),
         0, 0, 1;
    x(0) += v * dt * cos(x(2));
    x(1) += v * dt * sin(x(2));
    x(2) += omega * dt;
    P = F * P * F.transpose() + Q;
}

void EKF::update(Eigen::Vector2d z) {
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,3,2> K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix3d::Identity() - K * H) * P;
}

Eigen::Vector3d EKF::getState() { 
    return x; 
};
    
    