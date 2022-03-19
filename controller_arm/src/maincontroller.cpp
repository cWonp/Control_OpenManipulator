#include "../include/controller_arm/maincontroller.hpp"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "controller_arm");
    ros::NodeHandle n;

    n.getParam("/controller_arm/Run_RealRobot", Controller.Run_RealRobot);
    n.getParam("/controller_arm/Run_Gazebo", Controller.Run_Gazebo);

    /*****************************************************************************
    ** Message Init
    *****************************************************************************/
    //Publisher
    StateArmPub = n.advertise<controller_arm_msg::StateInfo>("StateInfo", 100);
    MonitorPub = n.advertise<controller_arm_msg::Monitor>("Monitor", 100);
    for(int i = 0; i < 6; i++){
        ros::Publisher pb;
        pb = n.advertise<std_msgs::Float64>("joint"+to_string(i+1)+"_position/command", 100);
        JointAngle2Gazebo_pub.push_back(pb);
    }

    //Subscriber
    GuiControlSub = n.subscribe("GuiControl", 1000, &MainController::GuiControlSub, &Controller);
    VisualControlSub = n.subscribe("VisualControl", 1000, &MainController::VisualControlSub, &Controller);

    /*****************************************************************************
    ** Robot Init
    *****************************************************************************/
    Controller.Solver.init();
    sleep(1);
    Controller.DXLinit();
    sleep(1);
    //cout<<Controller.ReadDXLPos()<<endl;
    Controller.init();
    sleep(1);

    //timer...................
    ros::Timer timer = n.createTimer(ros::Duration(tick), &MainController::TimerCallback, &Controller); //10ms

    sleep(1);
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

/*****************************************************************************
** Timer Callback function
*****************************************************************************/
void MainController::TimerCallback(const ros::TimerEvent&){
    if(Move.RunMode != STOP){
        Controller.execute(Move.RunMode);
        t++;

        if(t > Move.RunTime){
            t = 0;
            Move.RunMode = STOP;
            Controller.UsingAngLimit = EnableJointAngLimit;

            // Pub state to GUI
            StateArmMsg.MotionEndFlag = true;
            StateArmMsg.UIcmd = RUN;
            StateArmPub.publish(StateArmMsg);
            cout<<endl<<"**********************************************"<<endl;
            cout<<"                  Motion End                   "<<endl;
            cout<<"**********************************************"<<endl;
        }
    }
    else{
        t = 0;
    }
}

/*****************************************************************************
** Msg Callback function
*****************************************************************************/
void MainController::GuiControlSub(const controller_arm_msg::GuiControl::ConstPtr &msg){
    if(msg->UIcmd == RUN){ // Real running=======================================================================
        if(msg->RunFlag == POS_CTRL && Move.RunMode == STOP){ //.....................Pos control.....
            Move.RunMode = POS_CTRL;
            Move.OriMode = msg->InputOrientation;
            Move.RunTime = msg->EntTime*sec2tick;
            //Minimum RunTime
            if(Move.RunTime == 0.0) Move.RunTime = 1;   // STEP
            else if(Move.RunTime < RunTimeMin) Move.RunTime = RunTimeMin;

            //Getting From Info
            if(Solver.SolveFK(ArmState.JointAng, &Move.PosFrom, &Move.RotFrom)){
                Move.QuatFrom = Move.RotFrom;
                Move.OriEulFrom = Solver.RotMat2EulerAng(Move.RotFrom);
            }
            else{
                abort();
            }

            //INPUT POSITION...............................................
            if(msg->InputPosition == GlobalPos){
                Move.PosTo << msg->PosTarget.x, msg->PosTarget.y, msg->PosTarget.z;
            }
            else if(msg->InputPosition == LocalPos){
                Move.PosTo << Solver.ArmHW.PosInit(0)+msg->PosTarget_local.x, Solver.ArmHW.PosInit(1)+msg->PosTarget_local.y, Solver.ArmHW.PosInit(2)+msg->PosTarget_local.z;
            }
            else if(msg->InputPosition == Length){
                double LinearMotionLength = msg->LinearMotionLength;
                Move.PosTo = Solver.CalculateLinear(ArmState.Pos, ArmState.Rot, LinearMotionLength);
            }
            //INPUT ORIENTATION............................................
            if(Move.OriMode == RCM){
                Move.oriRCM << msg->PosRCM.x, msg->PosRCM.y, msg->PosRCM.z;
                Move.RotTo = Solver.GetRCMcsOrientation(Move.oriRCM, Move.PosTo);
                Move.OriEulTo = Solver.RotMat2EulerAng(Move.RotTo);
                Move.QuatTo = Move.RotTo;
            }
            else if(Move.OriMode == YPR){
                Move.OriEulTo << msg->OriTarget.x, msg->OriTarget.y, msg->OriTarget.z;
                Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
                Move.QuatTo = Move.RotTo;
            }
            else if(Move.OriMode == HeadingAng){
                Move.OriEulTo << 0, (PI/2)-msg->HeadingAngPitch, msg->HeadingAngYaw;
                Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
                Move.QuatTo = Move.RotTo;
            }
        }
        else if(msg->RunFlag == ANG_CTRL && Move.RunMode == STOP){//..................Ang control.....
            Move.RunMode = ANG_CTRL;
            Move.RunTime = msg->EntTime*sec2tick;
            if(Move.RunTime == 0.0) Move.RunTime = 1;
            else if(Move.RunTime < RunTimeMin) Move.RunTime = RunTimeMin;

            Move.JointAngFrom = ArmState.JointAng;

            for(int i = 0; i < 6; i++){
                Move.JointAngTo(i) = Solver.to2PI(msg->JointAngTarget[i]);
            }
        }
        else if(msg->RunFlag == MOTION_ROTATING && Move.RunMode == STOP){
            RotateMotion.num = msg->RotateNum;
            RotateMotion.time = msg->RotateTime*sec2tick;
            RotateMotion.ang = Solver.to2PI(msg->RotateAng);
            RotateMotion.entire_time = RotateMotion.num*RotateMotion.time;

            Move.RunMode = MOTION_ROTATING;
            Move.RunTime = RotateMotion.entire_time;
        }
        else if(msg->RunFlag == STEP_VelocityLimit /*&& Move.RunMode == STOP*/){
            Move.RunMode = STEP_VelocityLimit;
            Move.OriMode = YPR;
            Move.RunTime = 100;

            //Getting From Info
            if(Solver.SolveFK(ArmState.JointAng, &Move.PosFrom, &Move.RotFrom)){
                Move.QuatFrom = Move.RotFrom;
                Move.OriEulFrom = Solver.RotMat2EulerAng(Move.RotFrom);
            }
            else{
                abort();
            }

            //INPUT POSITION...............................................
            if(msg->InputPosition == GlobalPos){
                Move.PosTo << msg->PosTarget.x, msg->PosTarget.y, msg->PosTarget.z;
            }
            else if(msg->InputPosition == LocalPos){
                Move.PosTo << Solver.ArmHW.PosInit(0)+msg->PosTarget_local.x, Solver.ArmHW.PosInit(1)+msg->PosTarget_local.y, Solver.ArmHW.PosInit(2)+msg->PosTarget_local.z;
            }
            else if(msg->InputPosition == Length){
                double LinearMotionLength = msg->LinearMotionLength;
                Move.PosTo = Solver.CalculateLinear(ArmState.Pos, ArmState.Rot, LinearMotionLength);
            }
            //INPUT ORIENTATION............................................
            if(Move.OriMode == RCM){
                Move.oriRCM << msg->PosRCM.x, msg->PosRCM.y, msg->PosRCM.z;
                Move.RotTo = Solver.GetRCMcsOrientation(Move.oriRCM, Move.PosTo);
                Move.OriEulTo = Solver.RotMat2EulerAng(Move.RotTo);
                Move.QuatTo = Move.RotTo;
            }
            else if(Move.OriMode == YPR){
                Move.OriEulTo << msg->OriTarget.x, msg->OriTarget.y, msg->OriTarget.z;
                Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
                Move.QuatTo = Move.RotTo;
            }
            else if(Move.OriMode == HeadingAng){
                Move.OriEulTo << 0, (PI/2)-msg->HeadingAngPitch, msg->HeadingAngYaw;
                Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
                Move.QuatTo = Move.RotTo;
            }

            PosPrevious = Move.PosFrom;
            OriPrevious = Move.OriEulFrom;
        }
        //#if DEBUG
        cout<<endl<<"**********************************************"<<endl;
        cout<<"                 GUI Control                  "<<endl;
        cout<<"**********************************************"<<endl;
        cout<<"1) Move from POS     = "<<Move.PosFrom.transpose() << endl;
        cout<<"2) Move from ORI(Eul)= "<<Move.OriEulFrom.transpose()*R2D << endl;
        cout<<"3) Move from ORI(Mat)= "<<endl<<Move.RotFrom << endl <<endl;
        cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
        cout<<"1) Move to POS     = "<<Move.PosTo.transpose() << endl;
        cout<<"2) Move to ORI(Eul)= "<<Move.OriEulTo.transpose()*R2D << endl;
        cout<<"3) Move to ORI(Mat)= "<<endl<<Move.RotTo << endl <<endl;
        cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl<<endl;
        //#endif
    }
    else if(msg->UIcmd == GetFK){ // Just get FK result===============================================================
        Vector3d Pos = Vector3d::Zero();
        Matrix3d Rot = Matrix3d::Zero();
        Vector3d Ori = Vector3d::Zero();

        if(Solver.SolveFK(ArmState.JointAng, &Pos, &Rot)){
            Ori = Solver.RotMat2EulerAng(Rot);
        }
        else{
            abort();
        }
        StateArmMsg.PosFrom.x = Pos(0); StateArmMsg.PosFrom.y = Pos(1); StateArmMsg.PosFrom.z = Pos(2);
        StateArmMsg.OriFrom.x = Ori(0); StateArmMsg.OriFrom.y = Ori(1); StateArmMsg.OriFrom.z = Ori(2);

        StateArmMsg.UIcmd = GetFK;
        StateArmPub.publish(StateArmMsg);
    }
    else if(msg->UIcmd == ChangeInitPos){ // Changel Init position // Send Init Joint angle to=========================
        Vector3d Pos = Vector3d::Zero();
        Matrix3d Rot = Matrix3d::Zero();
        Vector3d Ori = Vector3d::Zero();
        VectorNd Ang = VectorNd::Zero(6);

        Pos << msg->PosInit.x, msg->PosInit.y, msg->PosInit.z;
        Ori << msg->OriInit.x, msg->OriInit.y, msg->OriInit.z;
        Rot = Solver.EulerAng2RotMat(Ori);

        cout<<"Change Init position : "<< Pos.transpose()<<endl;
        cout<<"Change Init Orientation : "<< Ori.transpose()<<endl;

        if(Solver.SolveIK(Pos, Rot, &Ang, ArmState.JointAng)){
            StateArmMsg.PosInit.x = Pos(0); StateArmMsg.PosInit.y = Pos(1); StateArmMsg.PosInit.z = Pos(2);
            StateArmMsg.OriInit.x = Ori(0); StateArmMsg.OriInit.y = Ori(1); StateArmMsg.OriInit.z = Ori(2);

            for(int i = 0; i < 6; i++){
                StateArmMsg.JointAngInit[i] = Solver.toPI(Ang(i));
            }

            StateArmMsg.UIcmd = ChangeInitPos;
            StateArmPub.publish(StateArmMsg);

            Solver.ArmHW.JointAngInit = Ang;
            Solver.ArmHW.PosInit = Pos;
            Solver.ArmHW.OriEulInit = Ori;
            Solver.ArmHW.RotInit = Rot;
        }
        else{
            cout<<endl<<"**********************************************"<<endl;
            cout<<"    [Error] Can not change Init Position!     "<<endl;
            cout<<"**********************************************"<<endl;
        }
    }
    else if(msg->UIcmd == ChnageInitAng){ // Change Init Ang============================================================
        Vector3d Pos = Vector3d::Zero();
        Matrix3d Rot = Matrix3d::Zero();
        Vector3d Ori = Vector3d::Zero();
        VectorNd Ang = VectorNd::Zero(6);
        for(int i = 0; i < 6; i++){
            Ang(i) = Solver.to2PI(msg->JointAngTarget[i]);
        }
        if(Solver.SolveFK(Ang, &Pos, &Rot)){
            Ori = Solver.RotMat2EulerAng(Rot);

            StateArmMsg.PosInit.x = Pos(0); StateArmMsg.PosInit.y = Pos(1); StateArmMsg.PosInit.z = Pos(2);
            StateArmMsg.OriInit.x = Solver.toPI(Ori(0)); StateArmMsg.OriInit.y = Solver.toPI(Ori(1)); StateArmMsg.OriInit.z = Solver.toPI(Ori(2));

            for(int i = 0; i < 6; i++){
                StateArmMsg.JointAngInit[i] = Solver.toPI(Ang(i));
            }

            StateArmMsg.UIcmd = ChnageInitAng;
            StateArmPub.publish(StateArmMsg);

            Solver.ArmHW.JointAngInit = Ang;
            Solver.ArmHW.PosInit = Pos;
            Solver.ArmHW.OriEulInit = Ori;
            Solver.ArmHW.RotInit = Rot;
        }
        else{
            cout<<endl<<"**********************************************"<<endl;
            cout<<"    [Error] Can not change Init Position!     "<<endl;
            cout<<"**********************************************"<<endl;
        }
    }
    else if(msg->UIcmd == LinearMotion){ // Linear Motion===============================================================
        double LinearMotionLength = msg->LinearMotionLength;
        Vector3d PosCalc;
        PosCalc = Solver.CalculateLinear(ArmState.Pos, ArmState.Rot, LinearMotionLength);

        Vector3d OriCalc;
        OriCalc = Solver.RotMat2EulerAng(ArmState.Rot);

        StateArmMsg.PosTo.x = PosCalc(0); StateArmMsg.PosTo.y = PosCalc(1); StateArmMsg.PosTo.z = PosCalc(2);
        StateArmMsg.OriTo.x = Solver.toPI(OriCalc(0)); StateArmMsg.OriTo.y = Solver.toPI(OriCalc(1)); StateArmMsg.OriTo.z = Solver.toPI(OriCalc(2));
        StateArmMsg.UIcmd = LinearMotion;
        StateArmPub.publish(StateArmMsg);
    }
}
void MainController::VisualControlSub(const controller_arm_msg::VisualControl::ConstPtr &msg){
    if(msg->RunFlag == POS_CTRL && Move.RunMode == STOP){ //.....................Pos control.....
        Move.RunTime = msg->Time*100;
        if(Move.RunTime == 0.0) Move.RunTime = 1;
        else if(Move.RunTime < RunTimeMin) Move.RunTime = RunTimeMin;

        //Getting From Info
        if(Solver.SolveFK(ArmState.JointAng, &Move.PosFrom, &Move.RotFrom)){
            Move.QuatFrom = Move.RotFrom;
            Move.OriEulFrom = Solver.RotMat2EulerAng(Move.RotFrom);
        }
        else{
            abort();
        }
        Move.PosTo << msg->PosX, msg->PosY, msg->PosZ;

        Move.OriEulTo << msg->Roll, msg->Pitch, msg->Yaw;
        Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
        Move.QuatTo = Move.RotTo;


        StateArmMsg.PosTo.x = Move.PosTo(0); StateArmMsg.PosTo.y = Move.PosTo(1); StateArmMsg.PosTo.z = Move.PosTo(2);
        StateArmMsg.OriTo.x = Move.OriEulTo(0); StateArmMsg.OriTo.y = Move.OriEulTo(1); StateArmMsg.OriTo.z = Move.OriEulTo(2);
        StateArmMsg.UIcmd = VisualCtrlPos;
        for(int i = 0; i < 5; i++){
            StateArmPub.publish(StateArmMsg);
        }

        Move.RunMode = POS_CTRL;
    }
    else if(msg->RunFlag == ANG_CTRL && Move.RunMode == STOP){//..................Ang control.....
        Move.RunTime = msg->Time*100;
        if(Move.RunTime == 0.0) Move.RunTime = 1;
        else if(Move.RunTime < RunTimeMin) Move.RunTime = RunTimeMin;

        Move.JointAngFrom = ArmState.JointAng;

        for(int i = 0; i < 6; i++){
            Move.JointAngTo(i) = Solver.to2PI(msg->JointAng[i]);
        }

        for(int i = 0; i < 6; i++){
            StateArmMsg.JointAngTo[i] = Solver.toPI(Move.JointAngTo(i));
        }
        StateArmMsg.UIcmd = VisualCtrlAng;
        for(int i = 0; i < 5; i++){
            StateArmPub.publish(StateArmMsg);
        }

        Move.RunMode = ANG_CTRL;
    }
/*

    Move.RunMode = STEP_VelocityLimit;
    Move.OriMode = YPR;
    Move.RunTime = 100;

    //Getting From Info
    if(Solver.SolveFK(ArmState.JointAng, &Move.PosFrom, &Move.RotFrom)){
        Move.QuatFrom = Move.RotFrom;
        Move.OriEulFrom = Solver.RotMat2EulerAng(Move.RotFrom);
    }
    else{
        abort();
    }

    Move.RunTime = 10;
    if(Move.RunTime == 0.0) Move.RunTime = 1;
    else if(Move.RunTime < RunTimeMin) Move.RunTime = RunTimeMin;

    //Getting From Info
    if(Solver.SolveFK(ArmState.JointAng, &Move.PosFrom, &Move.RotFrom)){
        Move.QuatFrom = Move.RotFrom;
        Move.OriEulFrom = Solver.RotMat2EulerAng(Move.RotFrom);
    }
    else{
        abort();
    }
    Move.PosTo << msg->PosX, msg->PosY, msg->PosZ;

    Move.OriEulTo << msg->Roll, msg->Pitch, msg->Yaw;
    Move.RotTo = Solver.EulerAng2RotMat(Move.OriEulTo);
    Move.QuatTo = Move.RotTo;


    StateArmMsg.PosTo.x = Move.PosTo(0); StateArmMsg.PosTo.y = Move.PosTo(1); StateArmMsg.PosTo.z = Move.PosTo(2);
    StateArmMsg.OriTo.x = Move.OriEulTo(0); StateArmMsg.OriTo.y = Move.OriEulTo(1); StateArmMsg.OriTo.z = Move.OriEulTo(2);
    StateArmMsg.UIcmd = VisualCtrlPos;
    for(int i = 0; i < 5; i++){
        StateArmPub.publish(StateArmMsg);
    }

    Move.RunMode = STEP_VelocityLimit;
    Move.OriMode = YPR;
    Move.RunTime = 100;


    PosPrevious = Move.PosFrom;
    OriPrevious = Move.OriEulFrom;
*/
}
/*****************************************************************************
** Execute (Making trajectory & Solving IK)
*****************************************************************************/
void MainController::execute(int RunMode){
    static bool Complete = false;
    cout << "[execute t = "<<t<<" ]"<<endl;

    if(RunMode == POS_CTRL){
        if(t == 0){ // Making trajectory at t==0.0
            // 1. Clear vector
            vector<TrajectoryPlanner::trajectory>().swap(PatternX.pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternY.pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternZ.pattern);
            vector<TrajectoryPlanner::trajectory_Slerp>().swap(PatternOrientation.SLERPpattern);

            // 2. Check target pos can solve IK
            VectorNd Ang = VectorNd::Zero(6);
            if(Solver.SolveIK(Move.PosTo, Move.RotTo, &Ang, ArmState.JointAng)){}
            else {
                Complete = false;
                t = Move.RunTime;
                cout<<"Error : Can not reach desired point !! "<<endl;
            }

            // 3. Make trajectory from present pos to target pos (Interpolation)
            PatternX.PutPoint(Linear, 0.0, Move.PosFrom(0));
            PatternX.PutPoint(Linear, Move.RunTime, Move.PosTo(0));

            PatternY.PutPoint(Linear, 0.0, Move.PosFrom(1));
            PatternY.PutPoint(Linear, Move.RunTime, Move.PosTo(1));

            PatternZ.PutPoint(Linear, 0.0, Move.PosFrom(2));
            PatternZ.PutPoint(Linear, Move.RunTime, Move.PosTo(2));

            //Orientation(Quat) use SLERP
            PatternOrientation.PutPoint(SLERP, 0.0, Move.QuatFrom);
            PatternOrientation.PutPoint(SLERP, Move.RunTime, Move.QuatTo);
        }

        armstate Res;
        Res.Pos(0) = PatternX.Result(t);
        Res.Pos(1) = PatternY.Result(t);
        Res.Pos(2) = PatternZ.Result(t);

        Res.JointAng = VectorNd::Zero(6);

        // if using RCM, Calculate RCM Orientation
        if(Move.OriMode == RCM){
            Res.Rot = Solver.GetRCMcsOrientation(Move.oriRCM, Res.Pos);
            Res.Quat = Res.Rot;
            Res.OriEul = Solver.RotMat2EulerAng(Res.Rot);
        }
        else{
            Res.Quat = PatternOrientation.SLERPResult(t);
            Res.Rot = Res.Quat.normalized().toRotationMatrix();
            Res.OriEul = Solver.RotMat2EulerAng(Res.Rot);
        }

        if(Solver.SolveIK(Res.Pos, Res.Rot, &Res.JointAng, ArmState.JointAng)){
            Complete = true;
            ArmState = Res;
        }
        else{
            Complete = false;
            cout << "[execute t = "<<t<<" ]"<<endl;
            t = Move.RunTime;
        }
    }
    else if(RunMode == ANG_CTRL){
        if(t == 0){
            // 1. Clear vector
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[0].pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[1].pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[2].pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[3].pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[4].pattern);
            vector<TrajectoryPlanner::trajectory>().swap(PatternAng[5].pattern);

            VectorNd Ang_new_from = VectorNd::Zero(6);
            VectorNd Ang_new_to = VectorNd::Zero(6);

            // 2. Fit the scale for interpolation
            for(int i = 0; i < 6; i++){
                Ang_new_from(i) = Solver.to2PI(Move.JointAngFrom(i) + PI);
                Ang_new_to(i) = Solver.to2PI(Move.JointAngTo(i) + PI);
            }

            // 3. Make trajectory (Interpolation)
            PatternAng[0].PutPoint(Linear, 0.0, Ang_new_from(0));
            PatternAng[0].PutPoint(Linear, Move.RunTime, Ang_new_to(0));

            PatternAng[1].PutPoint(Linear, 0.0, Ang_new_from(1));
            PatternAng[1].PutPoint(Linear, Move.RunTime, Ang_new_to(1));

            PatternAng[2].PutPoint(Linear, 0.0, Ang_new_from(2));
            PatternAng[2].PutPoint(Linear, Move.RunTime, Ang_new_to(2));

            PatternAng[3].PutPoint(Linear, 0.0, Ang_new_from(3));
            PatternAng[3].PutPoint(Linear, Move.RunTime, Ang_new_to(3));

            PatternAng[4].PutPoint(Linear, 0.0, Ang_new_from(4));
            PatternAng[4].PutPoint(Linear, Move.RunTime, Ang_new_to(4));

            PatternAng[5].PutPoint(Linear, 0.0, Ang_new_from(5));
            PatternAng[5].PutPoint(Linear, Move.RunTime, Ang_new_to(5));
        }

        for(int i = 0; i < 6; i++){
            double ResAng = Solver.to2PI(PatternAng[i].Result(t)-PI);

            if(UsingAngLimit){
                if(ResAng > Solver.ArmHW.JointAngLim_Min(i) && ResAng < Solver.ArmHW.JointAngLim_Max(i)){
                    cout << "Error : Joint angle Limit !!!" << endl;
                    Complete = false;
                    t = Move.RunTime;
                }
                else{
                    Complete = true;
                    ArmState.JointAng(i) = ResAng;
                }
            }
            else{
                Complete = true;
                ArmState.JointAng(i) = ResAng;
            }
        }

        if(Complete){
            if(Solver.SolveFK(ArmState.JointAng, &ArmState.Pos, &ArmState.Rot)){
                ArmState.OriEul = Solver.RotMat2EulerAng(ArmState.Rot);
                Complete = true;
            }
            else{
                Complete = false;
                t = Move.RunTime;
            }
        }
    }
    else if(RunMode == MOTION_ROTATING){
        if(t == 0){
            vector<TrajectoryPlanner::trajectory>().swap(PatternRotateMotion.pattern);
            PatternRotateMotion.PutPoint(Linear, 0.0, 0);
            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.25, RotateMotion.ang);

            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.25, RotateMotion.ang);
            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.5, 0);

            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.5, 0);
            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.75, -RotateMotion.ang);

            PatternRotateMotion.PutPoint(Linear, RotateMotion.time*0.75, -RotateMotion.ang);
            PatternRotateMotion.PutPoint(Linear, RotateMotion.time, 0);
        }

        ArmState.JointAng(5) = PatternRotateMotion.Result(t%RotateMotion.time);
        Complete = true;
    }
    else if(RunMode == STEP_VelocityLimit){

        armstate Res;

        PosThreshold[0] = PosThresholdX;
        PosThreshold[1] = PosThresholdY;
        PosThreshold[2] = PosThresholdZ;

        for(int i = 0; i < 3; i++){
            if(Move.PosTo(i) - PosPrevious(i) > PosThreshold[i]){
                Res.Pos(i) = ArmState.Pos(i) + PosThreshold[i];
            }
            else if(PosPrevious(i) - Move.PosTo(i) > PosThreshold[i]){
                Res.Pos(i) = ArmState.Pos(i) - PosThreshold[i];
            }
            else{
                Res.Pos(i) = Move.PosTo(i);
            }

            if(abs(Move.OriEulTo(i) - OriPrevious(i)) > OriThreshold){
                if(Move.OriEulTo(i) > PosPrevious(i))
                    Res.OriEul(i) = ArmState.OriEul(i) + OriThreshold;
                else if(Move.OriEulTo(i) < PosPrevious(i))
                    Res.OriEul(i) = ArmState.OriEul(i) - OriThreshold;
            }
            else{
                Res.OriEul(i) = Move.OriEulTo(i);
            }
        }
        Complete = true;

        Res.Rot = Solver.EulerAng2RotMat(Res.OriEul);
        Res.Quat = Res.Rot;

        //ArmState.Pos = Res.Pos;
        if(Solver.SolveIK(Res.Pos, Move.RotTo, &Res.JointAng, ArmState.JointAng)){
            Complete = true;
            ArmState = Res;
        }
        else{
            Complete = false;
            cout << "[execute t = "<<t<<" ]"<<endl;
            t = Move.RunTime;
        }

        cout<<"Move.PosTo = "<<Move.PosTo.transpose()<<endl;
        cout<<"PosPrevious = "<<PosPrevious.transpose()<<endl;
        cout<<"ArmState.Pos = "<<ArmState.Pos.transpose()<<endl;

        PosPrevious = ArmState.Pos;
        OriPrevious = ArmState.OriEul;
    }

    if(Complete){

        //Run Real Robot
        RunDynamixel(ArmState.JointAng);
        //Run Gazebo
        RunGazebo(ArmState.JointAng);

        //Gui update
        StateArmMsg.MotionEndFlag = false;
        StateArmMsg.PosNow.x = ArmState.Pos(0);
        StateArmMsg.PosNow.y = ArmState.Pos(1);
        StateArmMsg.PosNow.z = ArmState.Pos(2);
        StateArmMsg.OriNow.x = Solver.toPI(ArmState.OriEul(0));
        StateArmMsg.OriNow.y = Solver.toPI(ArmState.OriEul(1));
        StateArmMsg.OriNow.z = Solver.toPI(ArmState.OriEul(2));
        for(int i = 0; i < 6; i++){
            StateArmMsg.JointAngNow[i] = Solver.toPI(ArmState.JointAng(i));
        }
        StateArmMsg.UIcmd = RUN;
        StateArmPub.publish(StateArmMsg);

        if(Move.RunMode == STEP_VelocityLimit){
            if(ArmState.Pos == Move.PosTo /*&& ArmState.OriEul == Move.OriEulTo*/){
                Complete = false;
                t = Move.RunTime;
            }
            else{
                t = 0;
            }
        }

        //tf update
        BroadcastTF(ArmState.Pos, ArmState.OriEul);

        //erase~
        MonitorMsg.PosX = ArmState.Pos(0)*0.1;
        MonitorMsg.PosY = ArmState.Pos(1)*0.1;
        MonitorMsg.PosZ = ArmState.Pos(2)*0.1;
        MonitorMsg.OriX = ArmState.OriEul(0)*R2D;
        MonitorMsg.OriY = ArmState.OriEul(1)*R2D;
        MonitorMsg.OriZ = ArmState.OriEul(2)*R2D;
        MonitorMsg.Ang0 = ArmState.JointAng(0)*R2D;
        MonitorMsg.Ang1 = ArmState.JointAng(1)*R2D;
        MonitorMsg.Ang2 = ArmState.JointAng(2)*R2D;
        MonitorMsg.Ang3 = ArmState.JointAng(3)*R2D;
        MonitorMsg.Ang4 = ArmState.JointAng(4)*R2D;
        MonitorMsg.Ang5 = ArmState.JointAng(5)*R2D;
        MonitorPub.publish(MonitorMsg);
    }
}

/*****************************************************************************
** Init function
*****************************************************************************/
void MainController::init(){
    Move.JointAngTo = VectorNd::Zero(6);
    Move.JointAngFrom = VectorNd::Zero(6);
    ArmState.JointAng = VectorNd::Zero(6);

    Move.RunMode = STOP;

    ReadInitParam();

    //Get Init pos
    ArmState.JointAng = Solver.ArmHW.JointAngInit;
    ArmState.Pos = Solver.ArmHW.PosInit;
    ArmState.OriEul = Solver.ArmHW.OriEulInit;
    ArmState.Rot = Solver.ArmHW.RotInit;

    //Check Init Position
    if(Solver.SolveIK(ArmState.Pos, ArmState.Rot, &ArmState.JointAng, ArmState.JointAng)){}
    else{abort();}

    //Initial GUI
    StateArmMsg.MotionEndFlag = true;
    StateArmMsg.PosNow.x = ArmState.Pos(0);
    StateArmMsg.PosNow.y = ArmState.Pos(1);
    StateArmMsg.PosNow.z = ArmState.Pos(2);
    StateArmMsg.OriNow.x = Solver.toPI(ArmState.OriEul(0));
    StateArmMsg.OriNow.y = Solver.toPI(ArmState.OriEul(1));
    StateArmMsg.OriNow.z = Solver.toPI(ArmState.OriEul(2));
    for(int i = 0; i < 6; i++){
        StateArmMsg.JointAngNow.push_back(Solver.toPI(ArmState.JointAng(i)));
    }
    StateArmMsg.PosInit.x = ArmState.Pos(0);
    StateArmMsg.PosInit.y = ArmState.Pos(1);
    StateArmMsg.PosInit.z = ArmState.Pos(2);
    StateArmMsg.OriInit.x = Solver.toPI(ArmState.OriEul(0));
    StateArmMsg.OriInit.y = Solver.toPI(ArmState.OriEul(1));
    StateArmMsg.OriInit.z = Solver.toPI(ArmState.OriEul(2));
    for(int i = 0; i < 6; i++){
        StateArmMsg.JointAngInit.push_back(Solver.toPI(ArmState.JointAng(i)));
    }
    for(int i = 0; i < 6; i++){
        StateArmMsg.JointAngTo.push_back(Solver.toPI(ArmState.JointAng(i)));
    }
    StateArmMsg.UIcmd = RUN;
    StateArmPub.publish(StateArmMsg);

    if(Run_RealRobot){
        Move.RunTime = InitTime*sec2tick;
        cout<<"a"<<endl; //erase!
        Move.JointAngFrom = ReadDXLPos();
        Move.JointAngTo = ArmState.JointAng;
        Move.RunMode = ANG_CTRL;
        UsingAngLimit = false;
        cout<<"d"<<endl; //erase!
    }
    else{
        //Real robot init
        RunDynamixel(ArmState.JointAng);
        //Gazebo pub to init
        RunGazebo(ArmState.JointAng);
        //tf init
        BroadcastTF(ArmState.Pos, ArmState.OriEul);
    }
    cout<<endl<<"@ Init robot"<<endl;
}

bool MainController::DXLinit(){
    if(Run_RealRobot){
        cout<<"@ Dynamixel Init"<<endl;
        const char* port_name = "/dev/ttyUSB0";
        int baud_rate = 1000000;
        uint16_t model_number = 0;

        const char* log;
        bool result = false;

        result = DXL.init(port_name, baud_rate, &log);
        if(!result){
            cout<<"Failed to init : "<<log<<endl;
            return 0;
        }
        else    cout<<"Succeed to init "<<baud_rate<<endl;

        for(int i = 0; i < 6; i++){
            result = DXL.ping(Solver.ArmHW.DynamixelID[i], &model_number, &log);
            if(!result) cout<<"Failed to ping : "<<log<<endl;
            else cout<<"Succeeded to ping"<<endl<<"id : "<<Solver.ArmHW.DynamixelID[i]<<" model_number : "<<model_number<<endl;

            result = DXL.jointMode(Solver.ArmHW.DynamixelID[i], 0, 0, &log);
            if(!result) cout<<"Failed to change joint mode : "<<log<<endl;
            else    cout<<"Succeeded to change joint mode"<<endl;
        }

        result = DXL.addSyncWriteHandler(Solver.ArmHW.DynamixelID[0], "Goal_Position", &log);
        if(!result) cout<<"Failed to add sync write handler : "<<log<<endl;
        result = DXL.addSyncReadHandler(Solver.ArmHW.DynamixelID[0], "Present_Position", &log);
        if(!result) cout<<"Failed to add sync read handler : "<<log<<endl;
    }
}

bool MainController::ReadInitParam(){
    std::ifstream is;

    is.open("/home/ansurlab/catkin_ws/src/controller_arm/init/init");

    for(int i = 0; i < 6; i++){
        is>>Solver.ArmHW.JointAngInit(i);
    }
    for(int i = 0; i < 3; i++){
        is>>Solver.ArmHW.PosInit(i);
    }
    for(int i = 0; i < 3; i++){
        is>>Solver.ArmHW.OriEulInit(i);
    }

    is.close();

    Solver.ArmHW.RotInit = Solver.EulerAng2RotMat(Solver.ArmHW.OriEulInit);
}

/*****************************************************************************
** DYNAMIXEL function
*****************************************************************************/
VectorNd MainController::ReadDXLPos(){  // Read Dynamamixel
    if(Run_RealRobot){
        int32_t present_position[6] = {0, };
        VectorNd InitialAng = VectorNd::Zero(6);

        const char* log;
        bool result = false;

        for(int i = 0; i < 6; i++){
            int32_t* data;
            result = DXL.getPresentPositionData(Solver.ArmHW.DynamixelID[i], data, &log);
            if(!result){
                cout<<log<<endl;
            }
            else{
                present_position[i] = *data;
                cout<<"[ID "<<Solver.ArmHW.DynamixelID[i]<<"]  "<<present_position[i]<<endl;
                InitialAng(i) = (((present_position[i]*360)/4096)-180)*D2R;
                InitialAng(i) = Solver.to2PI(InitialAng(i));
                cout<<"[ID "<<i<<"]  ang(deg) "<<Solver.toPI(InitialAng(i))*R2D<<endl;
                cout<<"[ID "<<i<<"]  ang(rad) "<<Solver.toPI(InitialAng(i))<<endl;
            }
        }
        return InitialAng;
    }
}
void MainController::RunDynamixel(VectorNd Ang){    // Write Dynamixel
    if(Run_RealRobot){
        int32_t goal_position[6] = {2048, };

        const char* log;
        bool result = false;
        const uint8_t handler_index = 0;

        for(int i = 0; i < 6; i++){
            double motor = 4096*(Solver.to2PI(Ang(i))/(2*PI))+2048;
            if(motor >= 4096) motor -= 4096;
            if(motor < 0) motor += 4096;
            goal_position[i] = motor;
        }

        result = DXL.syncWrite(handler_index, &goal_position[0], &log);
        if(!result) cout<<"Failed to sync write position : "<<log<<endl;
    }
}

/*****************************************************************************
** To other node
*****************************************************************************/
void MainController::BroadcastTF(Vector3d Pos, Vector3d Ori){
    static tf::TransformBroadcaster br;
    tf::Transform World2EndE;
    World2EndE.setOrigin(tf::Vector3(Pos(0), Pos(1), Pos(2)));
    tf::Quaternion q;
    q.setRPY(Ori(0), Ori(1), Ori(2));
    World2EndE.setRotation(q);
    string strr;
    strr = "arm_endeffector";
    br.sendTransform(tf::StampedTransform(World2EndE, ros::Time::now(), "world", strr));
}
void MainController::RunGazebo(VectorNd Ang){
    if(Run_Gazebo){
        for (uint8_t i = 0; i < 6; i ++)
        {
            std_msgs::Float64 msg;
            msg.data = Solver.toPI(Ang(i));
            JointAngle2Gazebo_pub.at(i).publish(msg);
        }
    }
}
