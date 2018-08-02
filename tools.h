#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define PI 3.14159265

class Tools {
public:
    /**
     * Constructor.
     */
    Tools();
    
    /**
     * Destructor.
     */
    virtual ~Tools();
    
    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     * A helper method to find related angle between -PI and PI.
     */
    double AdjustAngle(double angle);
    
};

#endif /* TOOLS_H_ */
