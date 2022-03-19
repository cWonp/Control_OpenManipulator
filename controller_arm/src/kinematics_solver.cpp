#include "../include/controller_arm/kinematics_solver.hpp"

void KinematicsSolver::init(){
    //rbdl : Get model from urdf file in model folder
    model = new Model();

    if (!Addons::URDFReadFromFile ("/home/ansurlab/catkin_ws/src/controller_arm/model/SamplingArm.urdf", model, false)) {
        cerr << "Error loading model " << endl;
        abort();
    }

    cout << "== Get Robot Model ==========================="<<endl;
    cout << "Degree of freedom overview:" << endl;
    cout << Utils::GetModelDOFOverview(*model);

    cout << "Model Hierarchy:" << endl;
    cout << Utils::GetModelHierarchy(*model);

    // Initial robot info===================================================
    ArmHW.ToolLength = 0.165;
    ArmHW.DynamixelID[0] = 2;
    ArmHW.DynamixelID[1] = 1;
    ArmHW.DynamixelID[2] = 13;
    ArmHW.DynamixelID[3] = 14;
    ArmHW.DynamixelID[4] = 12;
    ArmHW.DynamixelID[5] = 11;

    //Can not reach this area ~~ Min < area < Max
    ArmHW.JointAngLim_Min << 14*D2R, 120*D2R, 120*D2R, 120*D2R, 120*D2R, 360*D2R;
    ArmHW.JointAngLim_Max << 253*D2R, 240*D2R, 240*D2R, 240*D2R, 240*D2R, 0*D2R;

    ArmHW.JointAngInit << 0.0, 0.0, to2PI(-PI/2.0),0.0, 0.0, 0.0;

    if(SolveFK(ArmHW.JointAngInit, &ArmHW.PosInit, &ArmHW.RotInit)){
        ArmHW.OriEulInit = RotMat2EulerAng(ArmHW.RotInit);
#if DEBUG
        cout<<endl<<"== Calculate Init of Robot ====================="<<endl;
        cout<<"0) init Ang = "<<ArmHW.JointAngInit.transpose()<<endl;
        cout<<"1) init pos = "<<ArmHW.PosInit.transpose()<<endl;
        cout<<"2) init ori = "<<ArmHW.OriEulInit.transpose()*R2D<<endl;
        cout<<"3) init rot = "<<endl<<ArmHW.RotInit<<endl;
#endif
    }
    else{
        cout<<"Can't solve init pos!!"<<endl;
        abort();
    }
}

KinematicsSolver::~KinematicsSolver(){
    delete model;
}

bool KinematicsSolver::SolveIK(Vector3d Tooltip, Matrix3d ori, VectorNd* JointAng , VectorNd InitalAng){
    Vector3d BasePos (0., 0., 0.);
    Vector3d Joint6Pos (0., 0., 0.);
    VectorNd q = VectorNd::Zero (model->q_size);
    VectorNd qres(q);
    *JointAng = VectorNd::Zero(6);

    UpdateKinematicsCustom (*model, &InitalAng, NULL, NULL);

    //Fix the Scale
    Tooltip << Tooltip(0)*mm2m, Tooltip(1)*mm2m, Tooltip(2)*mm2m;

    //Calculate Joint6Pos using TooltipPos
    Vector3d ToolInitialPos (0., 0., -ArmHW.ToolLength);
    Joint6Pos = Tooltip + ori*ToolInitialPos;


    InverseKinematicsConstraintSet cs;
    cs.AddFullConstraint(ID_j6, BasePos, Joint6Pos, ori.transpose());

    #if DEBUG
    cout<<endl<<"~~~~~ Solve Inverse Kinematics !! ~~~~~"<<endl;
    cout << "1) Input(Position)     = " << Tooltip.transpose() <<endl;
    cout << "2) Input(ROT)  = " <<endl<< ori <<endl;
    cout << "2) Input(RPY)  = " <<RotMat2EulerAng(ori).transpose()*R2D <<endl;
    cout << "3) RBDL Input(J6 pos)  = " << Joint6Pos.transpose() <<endl;
    #endif

    if(InverseKinematics(*model, InitalAng, cs, qres)){}
    else{
#if DEBUG
        cout<<endl<<"~~~~~ Solve Inverse Kinematics !! ~~~~~"<<endl;
        cout << "1) Input(Position)     = " << Tooltip.transpose() <<endl;
        cout << "2) Input(ROT)  = " <<endl<< ori <<endl;
        cout << "2) Input(RPY)  = " <<RotMat2EulerAng(ori).transpose()*R2D <<endl;
        cout << "3) RBDL Input(J6 pos)  = " << Joint6Pos.transpose() <<endl;
#endif
        cerr << "Error : Can not solve IK (Solve IK) !!!" << endl;
        return 0;
    }

    bool complete = false;

    bool UsingAngLimit = true;
    UsingAngLimit = EnableJointAngLimit;
    //Limit
    for(int i = 0; i < 6; i++){
        qres(i) = to2PI(qres(i));
        if(qres(i) > to2PI(ArmHW.JointAngLim_Min(i)) && qres(i) < to2PI(ArmHW.JointAngLim_Max(i))){
            if(UsingAngLimit){
                complete = false; // change -> false
                cout << "Error : Joint angle Limit !!! (Solve IK)" << endl;
                return 0;
            }
        }
        else if(isnan(qres(i))){
            cout << "Error : Result is Nan !!!"<<endl;
            complete = false;
            return 0;
        }
        else{
            complete = true;
            qres(i) = toPI(qres(i));
        }
    }

    if(complete){
        *JointAng = qres;
        //#if DEBUG
        cout << "4) Output(JointAng)    = " << (*JointAng).transpose()*R2D << endl << endl;
        //#endif
        return 1;
    }
    else{
        cout<<"return 0"<<endl;
        return 0;
    }


}


bool KinematicsSolver::SolveFK(VectorNd JointAng, Vector3d* Pos, Matrix3d* Ori){
    Vector3d BasePos (0., 0., 0.);

    Vector3d Joint6Pos = CalcBodyToBaseCoordinates (*model, JointAng, ID_j6, BasePos);
    Matrix3d Orientation = CalcBodyWorldOrientation (*model, JointAng, ID_j6, false).transpose();

    Vector3d TooltipPos (0., 0., 0.);
    Vector4d ToolInitialPos (0., 0., ArmHW.ToolLength, 1);

    MatrixNd R_BE = MatrixNd::Zero(4,4);
    R_BE << Orientation, Joint6Pos, 0, 0, 0, 1;

    Vector4d temp = R_BE*ToolInitialPos;
    TooltipPos << temp(0)*m2mm, temp(1)*m2mm, temp(2)*m2mm;

    *Pos = TooltipPos;
    *Ori = Orientation;

#if DEBUG
    cout<<"~~~~~ Solve Forward Kinematics !! ~~~~~"<<endl;
    cout << "1) Input(JointAng)     = " << JointAng.transpose()*R2D << endl;
    cout << "2) Output(j6 pos) = " << Joint6Pos.transpose() <<endl;
    cout << "3) Output(Position)    = " << (*Pos).transpose() <<endl;
    cout << "4) Output(ROT) = " << endl<< *Ori <<endl <<endl;
    cout << "5) Output(RPY) = " << RotMat2EulerAng(*Ori).transpose()*R2D <<endl <<endl;
#endif
    return 1;
}

bool KinematicsSolver::GetJointCartensianPos(VectorNd JointAng){
    Vector3d BasePos (0., 0., 0.);
    Vector3d JointPos[6];

    for(int i = 1; i < 7; i++){
        JointPos[i-1] = CalcBodyToBaseCoordinates (*model, JointAng, i, BasePos);
        cout<<"Joint ["<<i<<"] = "<<JointPos[i-1].transpose()<<endl;
    }
}

void KinematicsSolver::SetToolLength(double Length){
    ArmHW.ToolLength = Length;
    cout << "Set Tool Length = " << ArmHW.ToolLength << endl;
}

Matrix3d KinematicsSolver::GetRCMcsOrientation(Vector3d PosRCM, Vector3d PosTooltip){
    Vector3d OriVector (0., 0., 0.);
    Vector3d BaseVector (0., 0., 1.);

    OriVector << PosTooltip(0)-PosRCM(0), PosTooltip(1)-PosRCM(1), PosTooltip(2)-PosRCM(2);
    double magnitude_OV = sqrt(pow(OriVector(0),2)+pow(OriVector(1),2)+pow(OriVector(2),2));
    OriVector << OriVector(0)/magnitude_OV, OriVector(1)/magnitude_OV, OriVector(2)/magnitude_OV;

    Matrix3d ori = Matrix3d::Zero();
    ori = Eigen::Quaterniond::FromTwoVectors(BaseVector, OriVector).toRotationMatrix();
    return ori;
}

Vector3d KinematicsSolver::RotMat2EulerAng(Matrix3d Rot){
    Vector3d EulAng = Rot.eulerAngles(2,1,0);
    Vector3d resYPR;
    resYPR << EulAng(2), EulAng(1), EulAng(0);
    return resYPR;
}

Matrix3d KinematicsSolver::EulerAng2RotMat(Vector3d Eul){
    Eigen::Matrix3d Rot = Eigen::Matrix3d::Zero();
    Eigen::Vector3d E_Eul= Eigen::Vector3d::Zero();
    E_Eul << Eul(0), Eul(1), Eul(2);

    //Yaw Pitch Roll
    Rot = Eigen::AngleAxisd(E_Eul(2), Eigen::Vector3d::UnitZ())
            *Eigen::AngleAxisd(E_Eul(1), Eigen::Vector3d::UnitY())
            *Eigen::AngleAxisd(E_Eul(0), Eigen::Vector3d::UnitX());

    return Rot;
}

Eigen::Quaterniond KinematicsSolver::RPY2Quat(Vector3d RPY){
    Eigen::Quaterniond q;

    q = Eigen::AngleAxisd(RPY(2), Eigen::Vector3d::UnitZ())
            *Eigen::AngleAxisd(RPY(1), Eigen::Vector3d::UnitY())
            *Eigen::AngleAxisd(RPY(0), Eigen::Vector3d::UnitX());
    return q;
}

Vector3d KinematicsSolver::CalculateLinear(Vector3d pos, Matrix3d rot, double length){
    Vector3d base;
    base << 0,0,1;

    Vector3d orivec;
    double t = 0;
    Vector3d pos_desire;

    orivec = rot*base;
    t = sqrt(pow(length,2)/(pow(orivec(0),2)+pow(orivec(1),2)+pow(orivec(2),2)));

    if(length < 0)  t = -t;

    for(int i = 0; i < 3; i++){
        pos_desire(i) = pos(i)+orivec(i)*t;
    }

    return pos_desire;
}

double KinematicsSolver::toPI(double rad){
    while(rad <= -PI || rad > PI){
        if(rad > PI)   rad -= 2*PI;
        else if(rad <= -PI) rad += 2*PI;
    }
    return rad;
}
double KinematicsSolver::to2PI(double rad){
    while(rad < 0 || rad >= 2*PI){
        if(rad < 0) rad += 2*PI;
        else if(rad >= 2*PI) rad -= 2*PI;
    }
    return rad;
}
double KinematicsSolver::to180(double deg){
    while(deg > 180 || deg <= -180){
        if(deg > 180)   deg -= 360;
        else if(deg <= -180) deg += 360;
    }
    return deg;
}
double KinematicsSolver::to360(double deg){
    while(deg < 0 || deg >= 360){
        if(deg < 0) deg += 360;
        else if(deg >= 360) deg -= 360;
    }
    return deg;
}
