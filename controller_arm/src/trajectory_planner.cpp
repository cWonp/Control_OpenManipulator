#include "../include/controller_arm/trajectory_planner.hpp"

void TrajectoryPlanner::PutPoint(int traj_degree, double time, double pos, double vel, double acc){
    vpoint.push_back({time, pos, vel, acc, traj_degree});
    ComputePolynom();
}
void TrajectoryPlanner::ComputePolynom(){
    vector<trajectory>().swap(pattern);

    if(vpoint.size() >= 2){
        for(int i = 0; i < vpoint.size()-1; i++){
            pattern.push_back({Interpolation(vpoint[i], vpoint[i+1]), vpoint[i].time, vpoint[i+1].time});
        }
    }
}

void TrajectoryPlanner::PutPoint(int traj_degree, double time, Eigen::Quaterniond quat){
    vquat.push_back({time, quat});

    vector<trajectory_Slerp>().swap(SLERPpattern);
    if(vquat.size() >= 2){
        for(int i = 0; i < vquat.size()-1; i++){
            SLERPpattern.push_back({vquat[i].time, vquat[i+1].time, vquat[i].quat, vquat[i+1].quat});
        }
    }
}

vector<double> TrajectoryPlanner::Interpolation(point f, point f1){
    double p0 = f.pos;
    double v0 = f.vel;
    double a0 = f.acc;
    double t0 = f.time;

    double p1 = f1.pos;
    double v1 = f1.vel;
    double a1 = f1.acc;
    double t1 = f1.time;

    double T = t1-t0;

    if(f.degree == Linear)
    {
        v0 = (p1-p0)/T;
        v1 = v0;
    }

    vector<double> get_coefs;
    get_coefs.push_back(p0);
    get_coefs.push_back(v0);
    get_coefs.push_back(a0/2);
    get_coefs.push_back((20*(p1-p0)-(8*v1+12*v0)*T-(3*a1-a0)*T*T)/(2*T*T*T));
    get_coefs.push_back((30*(p0-p1)+(14*v1+16*v0)*T+(3*a1-2*a0)*T*T)/(2*T*T*T*T));
    get_coefs.push_back((12*(p1-p0)-6*T*(v1+v0)-(a1-a0)*T*T)/(2*T*T*T*T*T));

    return get_coefs;
}

double TrajectoryPlanner::Result(double t){
    int idx = 0;
    double PosRes = 0.0;
    double t0 = 0, t1 = 0;
    for(int i = 0; i < pattern.size(); i++){
        if(t >= pattern[i].time_start && t <= pattern[i].time_end)
            idx = i;
    }

    t0 = pattern[idx].time_start;
    t1 = pattern[idx].time_end;

    for(int i = 0; i < pattern[idx].coefs.size(); i++){
        PosRes += pattern[idx].coefs[i]*pow(t-t0,i);
    }

    vector<point>().swap(vpoint);

    return PosRes;
}
Eigen::Quaterniond TrajectoryPlanner::SLERPResult(double t){
    int idx = 0;
    Eigen::Quaterniond qres;
    double t0 = 0, t1 = 0;
    for(int i = 0; i < SLERPpattern.size(); i++){
        if(t >= SLERPpattern[i].time_start && t <= SLERPpattern[i].time_end)
            idx = i;
    }

    t0 = SLERPpattern[idx].time_start;
    t1 = SLERPpattern[idx].time_end;

    qres = SLERPpattern[idx].q1.slerp((t-t0)/(t1-t0),SLERPpattern[idx].q2);

    vector<quaternion>().swap(vquat);

    return qres;
}

