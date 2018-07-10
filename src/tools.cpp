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

    for (int i=0; i<4; i++) {
        double z = 0;
        int n = 0;
        for (int j=estimations.size()-1; j<estimations.size(); j++) {
        	double d = estimations[j][i] - ground_truth[j][i];
            d = d*d;
            z += d;
            n++;
        }
        rmse(i) = sqrt(z/(double)(n > 0 ? n : 1));
    }

    return rmse;
}
