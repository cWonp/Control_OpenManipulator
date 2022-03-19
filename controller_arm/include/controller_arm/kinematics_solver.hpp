#ifndef KINEMATICS_SOLVER_HPP_
#define KINEMATICS_SOLVER_HPP_

#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Logging.h>

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;

/**********************
 * RBDL using [m]
 * In node using [mm]
 * ********************/

#define DEBUG 0

#define ID_j6 6

#define PI acos(-1)
#define R2D 180.0/acos(-1)
#define D2R acos(-1)/180.0
#define m2mm    1000
#define mm2m    0.001

#define EnableJointAngLimit    1 // when unable angle limit chang to 0

struct RobotHardware{
    VectorNd JointAngInit = VectorNd::Zero(6);
    Vector3d PosInit = Vector3d::Zero();
    Vector3d OriEulInit = Vector3d::Zero();
    Matrix3d RotInit = Matrix3d::Zero();

    double ToolLength;

    VectorNd JointAngLim_Min = VectorNd::Zero(6);
    VectorNd JointAngLim_Max = VectorNd::Zero(6);

    int DynamixelID[6] = {0, };
};

class KinematicsSolver{
public:
    ~KinematicsSolver();

    void init();
    void SetToolLength(double Length);
    Matrix3d GetRCMcsOrientation(Vector3d PosRCM, Vector3d PosTooltip);
    Vector3d RotMat2EulerAng(Matrix3d Rot);
    Matrix3d EulerAng2RotMat(Vector3d Eul);
    Eigen::Quaterniond RPY2Quat(Vector3d RPY);
    Vector3d CalculateLinear(Vector3d pos, Matrix3d rot, double length);
    double toPI(double rad);
    double to2PI(double rad);
    double to180(double deg);
    double to360(double deg);

    bool SolveIK(Vector3d Tooltip, Matrix3d ori, VectorNd* JointAng, VectorNd InitialAng);
    bool SolveFK(VectorNd JointAng, Vector3d* TooltipPos, Matrix3d* Ori);

    bool GetJointCartensianPos(VectorNd JointAng);

    RobotHardware ArmHW;
private:
    Model* model;
};

#endif /* KINEMATICS_SOLVER_HPP_ */
