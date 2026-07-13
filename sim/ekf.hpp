#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>

class EKF {
public:
    EKF();
    void predict(double v, double omega, double dt);
    void update(Eigen::Vector2d z);
    Eigen::Vector3d getState();

private:
    Eigen::Vector3d x;
    Eigen::Matrix3d P;
    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix<double,2,3> H;

};

#endif // EKF_HPP