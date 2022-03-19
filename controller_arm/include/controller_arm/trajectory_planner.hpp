#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

#define Linear  1
#define SLERP   2 // Spherical Linear Interpolation
#define Fifth   5

class TrajectoryPlanner{
public:
    struct trajectory
    {
        vector<double> coefs;
        double time_start;
        double time_end;
    };
    vector<trajectory> pattern;

    struct trajectory_Slerp
    {
        double time_start;
        double time_end;
        Eigen::Quaterniond q1;
        Eigen::Quaterniond q2;
    };
    vector<trajectory_Slerp> SLERPpattern;

    void PutPoint(int traj_degree, double time, double pos, double vel = 0.0, double acc = 0.0);
    void PutPoint(int traj_degree, double time, Eigen::Quaterniond quat);
    double Result(double t);
    Eigen::Quaterniond SLERPResult(double t);
private:
    struct point{
        double time;
        double pos;
        double vel;
        double acc;
        int degree;
    };
    vector<point> vpoint;

    struct quaternion
    {
        double time;
        Eigen::Quaterniond quat;
    };
    vector<quaternion> vquat;

    void ComputePolynom();
    vector<double> Interpolation(point f, point f1);
};
#endif /* TRAJECTORY_PLANNER_HPP_ */
