#include "ekf.hpp"
#include <iostream>
#include <Eigen/Dense>

int main() {
    EKF ekf;
    ekf.predict(1.0, 0, 0.1);
    Eigen::Vector2d z(1.0, 1.0);
    ekf.update(z);
    std::cout << ekf.getState() << std::endl;
    return 0;
}
    