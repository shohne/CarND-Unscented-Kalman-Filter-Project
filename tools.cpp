#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    for (int j=0; j<estimations.size(); j++) {
        VectorXd d = estimations[j] - ground_truth[j];
        VectorXd d2 = d.cwiseProduct(d);
        rmse += d2;
    }
    rmse = rmse / (double)estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;

}


double Tools::AdjustAngle(double angle) {
    while (angle < -PI)  angle += 2*PI;
    while (angle >  PI)  angle -= 2*PI;
    return angle;
}
