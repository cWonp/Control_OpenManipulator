/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gui_controller_arm/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_controller_arm {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

//***************Pos [mm]  Ang [Deg]*****************
//*****Send to controller : Pos [mm]  Ang [rad]******

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(StateInfoSignal()), this, SLOT(CallBackStateInfo()));

    qnode.init();

    for(int i = 0; i < 6; i++){
        GuiControlMsg.JointAngTarget.push_back(0);
        GuiControlMsg.JointAngInit.push_back(0);
    }
}

void MainWindow::CallBackStateInfo(){
    StateArmInfo = qnode.StateArmInfo;

    StateArmInfo.OriNow.x = StateArmInfo.OriNow.x*R2D; StateArmInfo.OriNow.y = StateArmInfo.OriNow.y*R2D; StateArmInfo.OriNow.z = StateArmInfo.OriNow.z*R2D;
    for(int i = 0; i < 6; i++){
        StateArmInfo.JointAngNow[i] = StateArmInfo.JointAngNow[i]*R2D;
    }
    StateArmInfo.OriInit.x = StateArmInfo.OriInit.x*R2D; StateArmInfo.OriInit.y = StateArmInfo.OriInit.y*R2D; StateArmInfo.OriInit.z = StateArmInfo.OriInit.z*R2D;
    for(int i = 0; i < 6; i++){
        StateArmInfo.JointAngInit[i] = StateArmInfo.JointAngInit[i]*R2D;
    }
    StateArmInfo.OriFrom.x = StateArmInfo.OriFrom.x*R2D; StateArmInfo.OriFrom.y = StateArmInfo.OriFrom.y*R2D; StateArmInfo.OriFrom.z = StateArmInfo.OriFrom.z*R2D;
    StateArmInfo.OriTo.x = StateArmInfo.OriTo.x*R2D; StateArmInfo.OriTo.y = StateArmInfo.OriTo.y*R2D; StateArmInfo.OriTo.z = StateArmInfo.OriTo.z*R2D;
    for(int i = 0; i < 6; i++){
        StateArmInfo.JointAngTo[i] = StateArmInfo.JointAngTo[i]*R2D;
    }

    if(StateArmInfo.UIcmd == RUN){//==================================================================================
        if(StateArmInfo.MotionEndFlag){ // Motion stop
            MoveFlag = false;
            //set target init
            ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosNow.x);
            ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosNow.y);
            ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosNow.z);
            ui.doubleSpinBox_TargetOriX->setValue(StateArmInfo.OriNow.x);
            ui.doubleSpinBox_TargetOriY->setValue(StateArmInfo.OriNow.y);
            ui.doubleSpinBox_TargetOriZ->setValue(StateArmInfo.OriNow.z);
            //set joint init
            ui.doubleSpinBox_JointAng1->setValue(StateArmInfo.JointAngNow[0]);
            ui.doubleSpinBox_JointAng2->setValue(StateArmInfo.JointAngNow[1]);
            ui.doubleSpinBox_JointAng3->setValue(StateArmInfo.JointAngNow[2]);
            ui.doubleSpinBox_JointAng4->setValue(StateArmInfo.JointAngNow[3]);
            ui.doubleSpinBox_JointAng5->setValue(StateArmInfo.JointAngNow[4]);
            ui.doubleSpinBox_JointAng6->setValue(StateArmInfo.JointAngNow[5]);

            ui.lineEdit_MovingState->setText("Ready");
            ui.lineEdit_MovingState->setStyleSheet("border-style:outset;"
                                                   "border-width:4px;"
                                                   "border-color:white;"
                                                   "font: bold 14px");

            ui.pushButton_MovePosOrder->setText("MOVE");
            ui.pushButton_MoveJointControl->setText("MOVE");
            ui.pushButton_Step1_Execute->setText("EXECUTE");
            ui.pushButton_Step2_Execute->setText("EXECUTE");
            ui.pushButton_Step3_Execute->setText("EXECUTE");
            ui.pushButton_Step4_Execute->setText("EXECUTE");
            ui.pushButton_Step5_Execute->setText("EXECUTE");
        }
        else{ // Motion running
            MoveFlag = true;
            ui.lineEdit_MovingState->setText("MOVING");
            ui.lineEdit_MovingState->setStyleSheet("border-style:outset;"
                                                   "border-width:4px;"
                                                   "border-color:red;"
                                                   "font: bold 14px");
        }
        ui.lineEdit_StatePosX->setText(QString::number(StateArmInfo.PosNow.x));
        ui.lineEdit_StatePosY->setText(QString::number(StateArmInfo.PosNow.y));
        ui.lineEdit_StatePosZ->setText(QString::number(StateArmInfo.PosNow.z));
        ui.lineEdit_StateOriX->setText(QString::number(StateArmInfo.OriNow.x));
        ui.lineEdit_StateOriY->setText(QString::number(StateArmInfo.OriNow.y));
        ui.lineEdit_StateOriZ->setText(QString::number(StateArmInfo.OriNow.z));

        ui.lineEdit_StateJointAng1->setText(QString::number(StateArmInfo.JointAngNow[0]));
        ui.lineEdit_StateJointAng2->setText(QString::number(StateArmInfo.JointAngNow[1]));
        ui.lineEdit_StateJointAng3->setText(QString::number(StateArmInfo.JointAngNow[2]));
        ui.lineEdit_StateJointAng4->setText(QString::number(StateArmInfo.JointAngNow[3]));
        ui.lineEdit_StateJointAng5->setText(QString::number(StateArmInfo.JointAngNow[4]));
        ui.lineEdit_StateJointAng6->setText(QString::number(StateArmInfo.JointAngNow[5]));

        ui.lineEdit_InitPosX->setText(QString::number(StateArmInfo.PosInit.x));
        ui.lineEdit_InitPosY->setText(QString::number(StateArmInfo.PosInit.y));
        ui.lineEdit_InitPosZ->setText(QString::number(StateArmInfo.PosInit.z));
        ui.lineEdit_InitOriX->setText(QString::number(StateArmInfo.OriInit.x));
        ui.lineEdit_InitOriY->setText(QString::number(StateArmInfo.OriInit.y));
        ui.lineEdit_InitOriZ->setText(QString::number(StateArmInfo.OriInit.z));

        ui.lineEdit_InitJointAng1->setText(QString::number(StateArmInfo.JointAngInit[0]));
        ui.lineEdit_InitJointAng2->setText(QString::number(StateArmInfo.JointAngInit[1]));
        ui.lineEdit_InitJointAng3->setText(QString::number(StateArmInfo.JointAngInit[2]));
        ui.lineEdit_InitJointAng4->setText(QString::number(StateArmInfo.JointAngInit[3]));
        ui.lineEdit_InitJointAng5->setText(QString::number(StateArmInfo.JointAngInit[4]));
        ui.lineEdit_InitJointAng6->setText(QString::number(StateArmInfo.JointAngInit[5]));
    }
    else if(StateArmInfo.UIcmd == GetFK){//============================================================================
        ui.lineEdit_FromPosX->setText(QString::number(StateArmInfo.PosFrom.x));
        ui.lineEdit_FromPosY->setText(QString::number(StateArmInfo.PosFrom.y));
        ui.lineEdit_FromPosZ->setText(QString::number(StateArmInfo.PosFrom.z));
        ui.lineEdit_FromOriX->setText(QString::number(StateArmInfo.OriFrom.x));
        ui.lineEdit_FromOriY->setText(QString::number(StateArmInfo.OriFrom.y));
        ui.lineEdit_FromOriZ->setText(QString::number(StateArmInfo.OriFrom.z));

        ui.lineEdit_ToPosX->setText(QString::number(GuiControlMsg.PosTarget.x));
        ui.lineEdit_ToPosY->setText(QString::number(GuiControlMsg.PosTarget.y));
        ui.lineEdit_ToPosZ->setText(QString::number(GuiControlMsg.PosTarget.z));
        if(GuiControlMsg.InputOrientation == YPR){
            ui.lineEdit_ToOriX->setText(QString::number(GuiControlMsg.OriTarget.x*R2D));
            ui.lineEdit_ToOriY->setText(QString::number(GuiControlMsg.OriTarget.y*R2D));
            ui.lineEdit_ToOriZ->setText(QString::number(GuiControlMsg.OriTarget.z*R2D));
        }
        else if(GuiControlMsg.InputOrientation == RCM){//========================================================================
            ui.lineEdit_ToOriX->setText(QString::number(GuiControlMsg.PosRCM.x));
            ui.lineEdit_ToOriY->setText(QString::number(GuiControlMsg.PosRCM.y));
            ui.lineEdit_ToOriZ->setText(QString::number(GuiControlMsg.PosRCM.z));
        }
        else if(GuiControlMsg.InputOrientation == HeadingAng){//=================================================================
            ui.lineEdit_ToOriX->setText(QString::number(0));
            ui.lineEdit_ToOriY->setText(QString::number(90-GuiControlMsg.HeadingAngPitch*R2D));
            ui.lineEdit_ToOriZ->setText(QString::number(GuiControlMsg.HeadingAngYaw*R2D));
        }
    }
    else if(StateArmInfo.UIcmd == ChangeInitPos && StateArmInfo.UIcmd == ChangeInitPos){//==============================
        ui.lineEdit_InitPosX->setText(QString::number(StateArmInfo.PosInit.x));
        ui.lineEdit_InitPosY->setText(QString::number(StateArmInfo.PosInit.y));
        ui.lineEdit_InitPosZ->setText(QString::number(StateArmInfo.PosInit.z));
        ui.lineEdit_InitOriX->setText(QString::number(StateArmInfo.OriInit.x));
        ui.lineEdit_InitOriY->setText(QString::number(StateArmInfo.OriInit.y));
        ui.lineEdit_InitOriZ->setText(QString::number(StateArmInfo.OriInit.z));

        ui.lineEdit_InitJointAng1->setText(QString::number(StateArmInfo.JointAngInit[0]));
        ui.lineEdit_InitJointAng2->setText(QString::number(StateArmInfo.JointAngInit[1]));
        ui.lineEdit_InitJointAng3->setText(QString::number(StateArmInfo.JointAngInit[2]));
        ui.lineEdit_InitJointAng4->setText(QString::number(StateArmInfo.JointAngInit[3]));
        ui.lineEdit_InitJointAng5->setText(QString::number(StateArmInfo.JointAngInit[4]));
        ui.lineEdit_InitJointAng6->setText(QString::number(StateArmInfo.JointAngInit[5]));
    }
    else if(StateArmInfo.UIcmd == LinearMotion){//=======================================================================
        ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosTo.x);
        ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosTo.y);
        ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosTo.z);
        ui.doubleSpinBox_TargetOriX->setValue(StateArmInfo.OriTo.x);
        ui.doubleSpinBox_TargetOriY->setValue(StateArmInfo.OriTo.y);
        ui.doubleSpinBox_TargetOriZ->setValue(StateArmInfo.OriTo.z);
    }
    else if(StateArmInfo.UIcmd == VisualCtrlPos){//======================================================================
        ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosTo.x);
        ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosTo.y);
        ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosTo.z);
        ui.doubleSpinBox_TargetOriX->setValue(StateArmInfo.OriTo.x);
        ui.doubleSpinBox_TargetOriY->setValue(StateArmInfo.OriTo.y);
        ui.doubleSpinBox_TargetOriZ->setValue(StateArmInfo.OriTo.z);

        ui.lineEdit_FromPosX->setText(QString::number(StateArmInfo.PosFrom.x));
        ui.lineEdit_FromPosY->setText(QString::number(StateArmInfo.PosFrom.y));
        ui.lineEdit_FromPosZ->setText(QString::number(StateArmInfo.PosFrom.z));
        ui.lineEdit_FromOriX->setText(QString::number(StateArmInfo.OriFrom.x));
        ui.lineEdit_FromOriY->setText(QString::number(StateArmInfo.OriFrom.y));
        ui.lineEdit_FromOriZ->setText(QString::number(StateArmInfo.OriFrom.z));

        ui.lineEdit_ToPosX->setText(QString::number(StateArmInfo.PosTo.x));
        ui.lineEdit_ToPosY->setText(QString::number(StateArmInfo.PosTo.y));
        ui.lineEdit_ToPosZ->setText(QString::number(StateArmInfo.PosTo.z));

        ui.lineEdit_ToOriX->setText(QString::number(StateArmInfo.OriTo.x*R2D));
        ui.lineEdit_ToOriY->setText(QString::number(StateArmInfo.OriTo.y*R2D));
        ui.lineEdit_ToOriZ->setText(QString::number(StateArmInfo.OriTo.z*R2D));
    }
    else if(StateArmInfo.UIcmd == VisualCtrlAng){//=======================================================================
        ui.doubleSpinBox_JointAng1->setValue(StateArmInfo.JointAngTo[0]);
        ui.doubleSpinBox_JointAng2->setValue(StateArmInfo.JointAngTo[1]);
        ui.doubleSpinBox_JointAng3->setValue(StateArmInfo.JointAngTo[2]);
        ui.doubleSpinBox_JointAng4->setValue(StateArmInfo.JointAngTo[3]);
        ui.doubleSpinBox_JointAng5->setValue(StateArmInfo.JointAngTo[4]);
        ui.doubleSpinBox_JointAng6->setValue(StateArmInfo.JointAngTo[5]);
    }
}

MainWindow::~MainWindow() {}

//Pos order============================================================
//Target.........
void MainWindow::on_doubleSpinBox_TargetPosX_valueChanged(double arg1){
    GuiControlMsg.PosTarget.x = arg1;
    if(GuiControlMsg.InputPosition == LocalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosX_local->setValue((GuiControlMsg.PosTarget.x)-StateArmInfo.PosInit.x);
    }
}
void MainWindow::on_doubleSpinBox_TargetPosY_valueChanged(double arg1){
    GuiControlMsg.PosTarget.y = arg1;
    if(GuiControlMsg.InputPosition == LocalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosY_local->setValue((GuiControlMsg.PosTarget.y)-StateArmInfo.PosInit.y);
    }
}
void MainWindow::on_doubleSpinBox_TargetPosZ_valueChanged(double arg1){
    GuiControlMsg.PosTarget.z = arg1;
    if(GuiControlMsg.InputPosition == LocalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosZ_local->setValue((GuiControlMsg.PosTarget.z)-StateArmInfo.PosInit.z);
    }
}
void MainWindow::on_doubleSpinBox_TargetPosX_local_valueChanged(double arg1){
    GuiControlMsg.PosTarget_local.x = arg1;
    if(GuiControlMsg.InputPosition == GlobalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosInit.x+(GuiControlMsg.PosTarget_local.x));
    }
}
void MainWindow::on_doubleSpinBox_TargetPosY_local_valueChanged(double arg1){
    GuiControlMsg.PosTarget_local.y = arg1;
    if(GuiControlMsg.InputPosition == GlobalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosInit.y+(GuiControlMsg.PosTarget_local.y));
    }
}
void MainWindow::on_doubleSpinBox_TargetPosZ_local_valueChanged(double arg1){
    GuiControlMsg.PosTarget_local.z = arg1;
    if(GuiControlMsg.InputPosition == GlobalPos){ // Using local coordinate
        ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosInit.z+(GuiControlMsg.PosTarget_local.z));
    }
}
void MainWindow::on_doubleSpinBox_TargetOriX_valueChanged(double arg1){
    GuiControlMsg.OriTarget.x = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_TargetOriY_valueChanged(double arg1){
    GuiControlMsg.OriTarget.y = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_TargetOriZ_valueChanged(double arg1){
    GuiControlMsg.OriTarget.z = arg1*D2R;
}
void MainWindow::on_radioButton_EnableLocal_clicked(bool checked){
    if(checked){
        POSctrl_InputPos = LocalPos;
        ui.radioButton_EnableLocal->setText("Local");
    }
    else{
        POSctrl_InputPos = GlobalPos;
        ui.radioButton_EnableLocal->setText("Global");
    }
}
void MainWindow::on_doubleSpinBox_HeadingAng_Yaw_valueChanged(double arg1){
    GuiControlMsg.HeadingAngYaw = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_HeadingAng_Pitch_valueChanged(double arg1){
    GuiControlMsg.HeadingAngPitch = arg1*D2R;
}
//RCM............
void MainWindow::on_radioButton_YPRmode_clicked(bool checked){
    POSctrl_InputOri = YPR;
}
void MainWindow::on_radioButton_RCMmode_clicked(bool checked){
    POSctrl_InputOri = RCM;
}
void MainWindow::on_radioButton_HeadingAngmode_clicked(bool checked){
    POSctrl_InputOri = HeadingAng;
}

void MainWindow::on_doubleSpinBox_RCMPosX_valueChanged(double arg1){
    GuiControlMsg.PosRCM.x = arg1;
}
void MainWindow::on_doubleSpinBox_RCMPosY_valueChanged(double arg1){
    GuiControlMsg.PosRCM.y = arg1;
}
void MainWindow::on_doubleSpinBox_RCMPosZ_valueChanged(double arg1){
    GuiControlMsg.PosRCM.z = arg1;
}
//Move...........
void MainWindow::on_doubleSpinBox_PosControlTime_valueChanged(double arg1){
    PosTime = arg1;
}
void MainWindow::on_pushButton_initialize_pos_clicked(){
    //set target init
    ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosInit.x);
    ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosInit.y);
    ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosInit.z);
    ui.doubleSpinBox_TargetOriX->setValue(StateArmInfo.OriInit.x);
    ui.doubleSpinBox_TargetOriY->setValue(StateArmInfo.OriInit.y);
    ui.doubleSpinBox_TargetOriZ->setValue(StateArmInfo.OriInit.z);
}
void MainWindow::on_pushButton_Set_clicked(){
    // Just get current position from control_arm node
    GuiControlMsg.UIcmd = GetFK; // Set flag change!!!
    GuiControlPub.publish(GuiControlMsg);
}
void MainWindow::on_pushButton_MovePosOrder_clicked(){
    if(MoveFlag){
        cout<<"*** Already Moving now!! Plz wait!! ***" << endl;
    }
    else{
        MoveFlag = true;
        if(!VelocityLimitmode){
            GuiControlMsg.RunFlag = POS_CTRL;
            GuiControlMsg.EntTime = PosTime;
            GuiControlMsg.UIcmd = RUN;
            GuiControlMsg.InputPosition = POSctrl_InputPos;
            GuiControlMsg.InputOrientation = POSctrl_InputOri;
        }
        else if(VelocityLimitmode){
            GuiControlMsg.RunFlag = STEP_VelocityLimit;
            GuiControlMsg.EntTime = PosTime;
            GuiControlMsg.UIcmd = RUN;
            GuiControlMsg.InputPosition = GlobalPos;
            GuiControlMsg.InputOrientation = YPR;
        }
        GuiControlPub.publish(GuiControlMsg);
        Current_GuiControlMsg = GuiControlMsg;
        ui.pushButton_MovePosOrder->setText("executed..");
        ui.pushButton_MoveJointControl->setText("executed..");
    }
}
void MainWindow::on_radioButton_EnableVelocityLimit_clicked(bool checked){
    if(checked){
        VelocityLimitmode = 1;
    }
    else{
        VelocityLimitmode = 0;
    }
}
//Motion---------------------------------------------------------------
void MainWindow::on_doubleSpinBox_LinearLength_valueChanged(double arg1){
    GuiControlMsg.LinearMotionLength = arg1;
}
void MainWindow::on_pushButton_Set_Linear_clicked(){
    GuiControlMsg.UIcmd = LinearMotion;
    GuiControlPub.publish(GuiControlMsg);
}
//Save............
void MainWindow::on_pushButton_Set_Save1_clicked(){
    save1.PosX = GuiControlMsg.PosTarget.x;
    save1.PosY = GuiControlMsg.PosTarget.y;
    save1.PosZ = GuiControlMsg.PosTarget.z;
    save1.OriX = GuiControlMsg.OriTarget.x*R2D;
    save1.OriY = GuiControlMsg.OriTarget.y*R2D;
    save1.OriZ = GuiControlMsg.OriTarget.z*R2D;

    ui.lineEdit_SavePos1X->setText(QString::number(save1.PosX));
    ui.lineEdit_SavePos1Y->setText(QString::number(save1.PosY));
    ui.lineEdit_SavePos1Z->setText(QString::number(save1.PosZ));
    ui.lineEdit_SaveOri1X->setText(QString::number(save1.OriX));
    ui.lineEdit_SaveOri1Y->setText(QString::number(save1.OriY));
    ui.lineEdit_SaveOri1Z->setText(QString::number(save1.OriZ));
}
void MainWindow::on_pushButton_Get_Save1_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(save1.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(save1.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(save1.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(save1.OriX);
    ui.doubleSpinBox_TargetOriY->setValue(save1.OriY);
    ui.doubleSpinBox_TargetOriZ->setValue(save1.OriZ);
}
void MainWindow::on_pushButton_Set_Save2_clicked(){
    save2.PosX = GuiControlMsg.PosTarget.x;
    save2.PosY = GuiControlMsg.PosTarget.y;
    save2.PosZ = GuiControlMsg.PosTarget.z;
    save2.OriX = GuiControlMsg.OriTarget.x*R2D;
    save2.OriY = GuiControlMsg.OriTarget.y*R2D;
    save2.OriZ = GuiControlMsg.OriTarget.z*R2D;

    ui.lineEdit_SavePos2X->setText(QString::number(save2.PosX));
    ui.lineEdit_SavePos2Y->setText(QString::number(save2.PosY));
    ui.lineEdit_SavePos2Z->setText(QString::number(save2.PosZ));
    ui.lineEdit_SaveOri2X->setText(QString::number(save2.OriX));
    ui.lineEdit_SaveOri2Y->setText(QString::number(save2.OriY));
    ui.lineEdit_SaveOri2Z->setText(QString::number(save2.OriZ));
}
void MainWindow::on_pushButton_Get_Save2_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(save2.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(save2.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(save2.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(save2.OriX);
    ui.doubleSpinBox_TargetOriY->setValue(save2.OriY);
    ui.doubleSpinBox_TargetOriZ->setValue(save2.OriZ);
}
//Joint order==========================================================
void MainWindow::on_doubleSpinBox_JointAng1_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[0] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointAng2_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[1] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointAng3_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[2] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointAng4_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[3] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointAng5_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[4] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointAng6_valueChanged(double arg1){
    GuiControlMsg.JointAngTarget[5] = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_JointControlTime_valueChanged(double arg1){
    AngTime = arg1;
}
void MainWindow::on_pushButton_MoveJointControl_clicked(){
    if(MoveFlag){
        cout<<"*** Already Moving now!! Plz wait!! ***" << endl;
    }
    else{
        MoveFlag = true;
        GuiControlMsg.RunFlag = ANG_CTRL;
        GuiControlMsg.EntTime = AngTime;
        GuiControlMsg.UIcmd = RUN;
        GuiControlPub.publish(GuiControlMsg);
        Current_GuiControlMsg = GuiControlMsg;
        ui.pushButton_MoveJointControl->setText("executed..");
        ui.pushButton_MovePosOrder->setText("executed..");
    }
}
void MainWindow::on_pushButton_initialize_ang_clicked(){
    ui.doubleSpinBox_JointAng1->setValue(StateArmInfo.JointAngInit[0]);
    ui.doubleSpinBox_JointAng2->setValue(StateArmInfo.JointAngInit[1]);
    ui.doubleSpinBox_JointAng3->setValue(StateArmInfo.JointAngInit[2]);
    ui.doubleSpinBox_JointAng4->setValue(StateArmInfo.JointAngInit[3]);
    ui.doubleSpinBox_JointAng5->setValue(StateArmInfo.JointAngInit[4]);
    ui.doubleSpinBox_JointAng6->setValue(StateArmInfo.JointAngInit[5]);
}

//Init=====================================================================
void MainWindow::on_pushButton_MoveInit_clicked(){
    //set target init
    ui.doubleSpinBox_TargetPosX->setValue(StateArmInfo.PosInit.x);
    ui.doubleSpinBox_TargetPosY->setValue(StateArmInfo.PosInit.y);
    ui.doubleSpinBox_TargetPosZ->setValue(StateArmInfo.PosInit.z);
    ui.doubleSpinBox_TargetOriX->setValue(StateArmInfo.OriInit.x);
    ui.doubleSpinBox_TargetOriY->setValue(StateArmInfo.OriInit.y);
    ui.doubleSpinBox_TargetOriZ->setValue(StateArmInfo.OriInit.z);

    ui.doubleSpinBox_JointAng1->setValue(StateArmInfo.JointAngInit[0]);
    ui.doubleSpinBox_JointAng2->setValue(StateArmInfo.JointAngInit[1]);
    ui.doubleSpinBox_JointAng3->setValue(StateArmInfo.JointAngInit[2]);
    ui.doubleSpinBox_JointAng4->setValue(StateArmInfo.JointAngInit[3]);
    ui.doubleSpinBox_JointAng5->setValue(StateArmInfo.JointAngInit[4]);
    ui.doubleSpinBox_JointAng6->setValue(StateArmInfo.JointAngInit[5]);

    if(MoveFlag){
        cout<<"*** Already Moving now!! Plz wait!! ***" << endl;
    }
    else{
        MoveFlag = true;
        GuiControlMsg.RunFlag = ANG_CTRL;
        GuiControlMsg.EntTime = 3;
        GuiControlMsg.UIcmd = RUN;
        GuiControlPub.publish(GuiControlMsg);
        Current_GuiControlMsg = GuiControlMsg;
    }
}
void MainWindow::on_pushButton_SetInit_pos_clicked(){
    if(MoveFlag){
        cout<<"*** Moving now!! Plz wait!! ***" << endl;
    }
    else{
        GuiControlMsg.PosInit.x = GuiControlMsg.PosTarget.x;
        GuiControlMsg.PosInit.y = GuiControlMsg.PosTarget.y;
        GuiControlMsg.PosInit.z = GuiControlMsg.PosTarget.z;
        GuiControlMsg.OriInit.x = GuiControlMsg.OriTarget.x;
        GuiControlMsg.OriInit.y = GuiControlMsg.OriTarget.y;
        GuiControlMsg.OriInit.z = GuiControlMsg.OriTarget.z;

        GuiControlMsg.UIcmd = ChangeInitPos;
        GuiControlPub.publish(GuiControlMsg);
    }
}
void MainWindow::on_pushButton_SetInit_ang_clicked(){
    for(int i = 0; i < 6; i++){
        GuiControlMsg.JointAngInit[i] = Current_GuiControlMsg.JointAngTarget[i];
    }
    GuiControlMsg.UIcmd = ChnageInitAng;
    GuiControlPub.publish(GuiControlMsg);
}
void MainWindow::on_pushButton_SaveFile_clicked(){
    //    QString fileName = QFileDialog::getSaveFileName(this,
    //                                                    tr("Save file"), "/home/ansurlab/catkin_ws/src/controller_arm/init");
    QString fileName = "/home/ansurlab/catkin_ws/src/controller_arm/init/init";

    if(fileName.isEmpty() == true)
        qDebug() << "Save Cancel";
    else{
        QFile *file = new QFile;
        file->setFileName(fileName);
        file->open(QIODevice::WriteOnly);
        QTextStream out(file);

        out<<StateArmInfo.JointAngInit[0]*D2R<<endl
                                            <<StateArmInfo.JointAngInit[1]*D2R<<endl
                                           <<StateArmInfo.JointAngInit[2]*D2R<<endl
                                          <<StateArmInfo.JointAngInit[3]*D2R<<endl
                                         <<StateArmInfo.JointAngInit[4]*D2R<<endl
                                        <<StateArmInfo.JointAngInit[5]*D2R<<endl

                                       <<StateArmInfo.PosInit.x<<endl
                                      <<StateArmInfo.PosInit.y<<endl
                                     <<StateArmInfo.PosInit.z<<endl

                                    <<StateArmInfo.OriInit.x*D2R<<endl
                                   <<StateArmInfo.OriInit.y*D2R<<endl
                                  <<StateArmInfo.OriInit.z*D2R<<endl
                                 <<",";

        file->close();
    }
}


//Sequence control======================================================
//STEP1 Reach the nostril.....
void MainWindow::on_doubleSpinBox_Step1PosX_valueChanged(double arg1){
    STEP1.PosX = arg1;
}
void MainWindow::on_doubleSpinBox_Step1PosY_valueChanged(double arg1){
    STEP1.PosY = arg1;
}
void MainWindow::on_doubleSpinBox_Step1PosZ_valueChanged(double  arg1){
    STEP1.PosZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step1OriR_valueChanged(double arg1){
    STEP1.OriX = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step1OriP_valueChanged(double arg1){
    STEP1.OriY = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step1OriY_valueChanged(double arg1){
    STEP1.OriZ = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step1Time_valueChanged(double arg1){
    STEP1_Time = arg1;
}
void MainWindow::on_pushButton_Step1_Set_clicked(){
    STEP1.PosX = GuiControlMsg.PosTarget.x;
    STEP1.PosY = GuiControlMsg.PosTarget.y;
    STEP1.PosZ = GuiControlMsg.PosTarget.z;
    STEP1.OriX = GuiControlMsg.OriTarget.x;
    STEP1.OriY = GuiControlMsg.OriTarget.y;
    STEP1.OriZ = GuiControlMsg.OriTarget.z;

    ui.doubleSpinBox_Step1PosX->setValue(STEP1.PosX);
    ui.doubleSpinBox_Step1PosY->setValue(STEP1.PosY);
    ui.doubleSpinBox_Step1PosZ->setValue(STEP1.PosZ);
    ui.doubleSpinBox_Step1OriR->setValue(STEP1.OriX*R2D);
    ui.doubleSpinBox_Step1OriP->setValue(STEP1.OriY*R2D);
    ui.doubleSpinBox_Step1OriY->setValue(STEP1.OriZ*R2D);
}
void MainWindow::on_pushButton_Step1_Get_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(STEP1.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(STEP1.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(STEP1.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(STEP1.OriX*R2D);
    ui.doubleSpinBox_TargetOriY->setValue(STEP1.OriY*R2D);
    ui.doubleSpinBox_TargetOriZ->setValue(STEP1.OriZ*R2D);
}
void MainWindow::on_pushButton_Step1_Execute_clicked(){
    GuiControlMsg.PosTarget.x = STEP1.PosX;
    GuiControlMsg.PosTarget.y = STEP1.PosY;
    GuiControlMsg.PosTarget.z = STEP1.PosZ;
    GuiControlMsg.OriTarget.x = STEP1.OriX;
    GuiControlMsg.OriTarget.y = STEP1.OriY;
    GuiControlMsg.OriTarget.z = STEP1.OriZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP1_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = YPR; //change -> YPR!!!
    GuiControlMsg.InputPosition = GlobalPos;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step1_Execute->setText("Running..");
}
//STEP2_term.....
void MainWindow::on_doubleSpinBox_Step2PosX_term_valueChanged(double arg1){
    STEP2_term.PosX = arg1;
}
void MainWindow::on_doubleSpinBox_Step2PosY_term_valueChanged(double arg1){
    STEP2_term.PosY = arg1;
}
void MainWindow::on_doubleSpinBox_Step2PosZ_term_valueChanged(double arg1){
    STEP2_term.PosZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step2OriR_term_valueChanged(double arg1){
    STEP2_term.OriX = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step2OriP_term_valueChanged(double arg1){
    STEP2_term.OriY = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step2OriY_term_valueChanged(double arg1){
    STEP2_term.OriZ = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step2Time_term_valueChanged(double arg1){
    STEP2_term_Time = arg1;
}

void MainWindow::on_pushButton_Step2_Set_term_clicked(){
    STEP2_term.PosX = GuiControlMsg.PosTarget.x;
    STEP2_term.PosY = GuiControlMsg.PosTarget.y;
    STEP2_term.PosZ = GuiControlMsg.PosTarget.z;
    STEP2_term.OriX = GuiControlMsg.OriTarget.x;
    STEP2_term.OriY = GuiControlMsg.OriTarget.y;
    STEP2_term.OriZ = GuiControlMsg.OriTarget.z;

    ui.doubleSpinBox_Step2PosX_term->setValue(STEP2_term.PosX);
    ui.doubleSpinBox_Step2PosY_term->setValue(STEP2_term.PosY);
    ui.doubleSpinBox_Step2PosZ_term->setValue(STEP2_term.PosZ);
    ui.doubleSpinBox_Step2OriR_term->setValue(STEP2_term.OriX*R2D);
    ui.doubleSpinBox_Step2OriP_term->setValue(STEP2_term.OriY*R2D);
    ui.doubleSpinBox_Step2OriY_term->setValue(STEP2_term.OriZ*R2D);
}
void MainWindow::on_pushButton_Step2_Get_term_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(STEP2_term.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(STEP2_term.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(STEP2_term.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(STEP2_term.OriX*R2D);
    ui.doubleSpinBox_TargetOriY->setValue(STEP2_term.OriY*R2D);
    ui.doubleSpinBox_TargetOriZ->setValue(STEP2_term.OriZ*R2D);
}
void MainWindow::on_pushButton_Step2_Execute_term_clicked(){
    GuiControlMsg.PosTarget.x = STEP2_term.PosX;
    GuiControlMsg.PosTarget.y = STEP2_term.PosY;
    GuiControlMsg.PosTarget.z = STEP2_term.PosZ;
    GuiControlMsg.OriTarget.x = STEP2_term.OriX;
    GuiControlMsg.OriTarget.y = STEP2_term.OriY;
    GuiControlMsg.OriTarget.z = STEP2_term.OriZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP2_term_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = YPR; //change -> YPR!!!
    GuiControlMsg.InputPosition = GlobalPos;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step2_Execute_term->setText("Running..");
}
//STEP2 Insert swab.....
void MainWindow::on_doubleSpinBox_Step2Length_valueChanged(double arg1){
    STEP2.length = arg1;
}
void MainWindow::on_doubleSpinBox_Step2RCMX_valueChanged(double arg1){
    STEP2.RCMX = arg1;
}
void MainWindow::on_doubleSpinBox_Step2RCMY_valueChanged(double arg1){
    STEP2.RCMY = arg1;
}
void MainWindow::on_doubleSpinBox_Step2RCMZ_valueChanged(double arg1){
    STEP2.RCMZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step2Time_valueChanged(double arg1){
    STEP2_Time = arg1;
}

void MainWindow::on_pushButton_Step2_Set_clicked(){
    STEP2.length = GuiControlMsg.LinearMotionLength;
    STEP2.RCMX = GuiControlMsg.PosRCM.x;
    STEP2.RCMY = GuiControlMsg.PosRCM.y;
    STEP2.RCMZ = GuiControlMsg.PosRCM.z;

    ui.doubleSpinBox_Step2Length->setValue(STEP2.length);
    ui.doubleSpinBox_Step2RCMX->setValue(STEP2.RCMX);
    ui.doubleSpinBox_Step2RCMY->setValue(STEP2.RCMY);
    ui.doubleSpinBox_Step2RCMZ->setValue(STEP2.RCMZ);
}
void MainWindow::on_pushButton_Step2_Get_clicked(){
    ui.doubleSpinBox_LinearLength->setValue(STEP2.length);

    ui.doubleSpinBox_RCMPosX->setValue(STEP2.RCMX);
    ui.doubleSpinBox_RCMPosY->setValue(STEP2.RCMY);
    ui.doubleSpinBox_RCMPosZ->setValue(STEP2.RCMZ);
}
void MainWindow::on_pushButton_Step2_Execute_clicked(){
    GuiControlMsg.LinearMotionLength = STEP2.length;
    GuiControlMsg.PosRCM.x = STEP2.RCMX;
    GuiControlMsg.PosRCM.y = STEP2.RCMY;
    GuiControlMsg.PosRCM.z = STEP2.RCMZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP2_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = RCM;
    GuiControlMsg.InputPosition = Length;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step2_Execute->setText("Running..");
}
//STEP3 Rotate.....
void MainWindow::on_doubleSpinBox_Step3Number_valueChanged(double arg1){
    STEP3.number = arg1;
    ui.lineEdit_STEP3EntireTime->setText(QString::number(STEP3.number*STEP3.time));
}
void MainWindow::on_doubleSpinBox_Step3Ang_valueChanged(double arg1){
    STEP3.angle = arg1;
}
void MainWindow::on_doubleSpinBox_Step3Time_valueChanged(double arg1){
    STEP3.time = arg1;
    ui.lineEdit_STEP3EntireTime->setText(QString::number(STEP3.number*STEP3.time));
}
void MainWindow::on_pushButton_Step3_Execute_clicked(){
    MoveFlag = true;
    GuiControlMsg.RunFlag = MOTION_ROTATING;
    GuiControlMsg.EntTime = STEP3.time*STEP3.number;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.RotateNum = STEP3.number;
    GuiControlMsg.RotateAng = STEP3.angle*D2R;
    GuiControlMsg.RotateTime = STEP3.time;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step3_Execute->setText("Running..");
}
//STEP4 Withdraw swab.....
void MainWindow::on_doubleSpinBox_Step4Length_valueChanged(double arg1){
    STEP4.length = arg1;
}
void MainWindow::on_doubleSpinBox_Step4RCMX_valueChanged(double arg1){
    STEP4.RCMX = arg1;
}
void MainWindow::on_doubleSpinBox_Step4RCMY_valueChanged(double arg1){
    STEP4.RCMY = arg1;
}
void MainWindow::on_doubleSpinBox_Step4RCMZ_valueChanged(double arg1){
    STEP4.RCMZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step4Time_valueChanged(double arg1){
    STEP4_Time = arg1;
}

void MainWindow::on_pushButton_Step4_Set_clicked(){
    STEP4.length = GuiControlMsg.LinearMotionLength;
    STEP4.RCMX = GuiControlMsg.PosRCM.x;
    STEP4.RCMY = GuiControlMsg.PosRCM.y;
    STEP4.RCMZ = GuiControlMsg.PosRCM.z;

    ui.doubleSpinBox_Step4Length->setValue(STEP4.length);
    ui.doubleSpinBox_Step4RCMX->setValue(STEP4.RCMX);
    ui.doubleSpinBox_Step4RCMY->setValue(STEP4.RCMY);
    ui.doubleSpinBox_Step4RCMZ->setValue(STEP4.RCMZ);
}
void MainWindow::on_pushButton_Step4_Get_clicked(){
    ui.doubleSpinBox_LinearLength->setValue(STEP4.length);

    ui.doubleSpinBox_RCMPosX->setValue(STEP4.RCMX);
    ui.doubleSpinBox_RCMPosY->setValue(STEP4.RCMY);
    ui.doubleSpinBox_RCMPosZ->setValue(STEP4.RCMZ);
}
void MainWindow::on_pushButton_Step4_Execute_clicked(){
    GuiControlMsg.LinearMotionLength = STEP4.length;
    GuiControlMsg.PosRCM.x = STEP4.RCMX;
    GuiControlMsg.PosRCM.y = STEP4.RCMY;
    GuiControlMsg.PosRCM.z = STEP4.RCMZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP4_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = RCM;
    GuiControlMsg.InputPosition = Length;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step4_Execute->setText("Running..");
}
//STEP4_term.....
void MainWindow::on_doubleSpinBox_Step4PosX_term_valueChanged(double arg1){
    STEP4_term.PosX = arg1;
}
void MainWindow::on_doubleSpinBox_Step4PosY_term_valueChanged(double arg1){
    STEP4_term.PosY = arg1;
}
void MainWindow::on_doubleSpinBox_Step4PosZ_term_valueChanged(double arg1){
    STEP4_term.PosZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step4OriR_term_valueChanged(double arg1){
    STEP4_term.OriX = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step4OriP_term_valueChanged(double arg1){
    STEP4_term.OriY = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step4OriY_term_valueChanged(double arg1){
    STEP4_term.OriZ = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step4Time_term_valueChanged(double arg1){
    STEP4_term_Time = arg1;
}

void MainWindow::on_pushButton_Step4_Set_term_clicked(){
    STEP4_term.PosX = GuiControlMsg.PosTarget.x;
    STEP4_term.PosY = GuiControlMsg.PosTarget.y;
    STEP4_term.PosZ = GuiControlMsg.PosTarget.z;
    STEP4_term.OriX = GuiControlMsg.OriTarget.x;
    STEP4_term.OriY = GuiControlMsg.OriTarget.y;
    STEP4_term.OriZ = GuiControlMsg.OriTarget.z;

    ui.doubleSpinBox_Step4PosX_term->setValue(STEP4_term.PosX);
    ui.doubleSpinBox_Step4PosY_term->setValue(STEP4_term.PosY);
    ui.doubleSpinBox_Step4PosZ_term->setValue(STEP4_term.PosZ);
    ui.doubleSpinBox_Step4OriR_term->setValue(STEP4_term.OriX*R2D);
    ui.doubleSpinBox_Step4OriP_term->setValue(STEP4_term.OriY*R2D);
    ui.doubleSpinBox_Step4OriY_term->setValue(STEP4_term.OriZ*R2D);
}
void MainWindow::on_pushButton_Step4_Get_term_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(STEP4_term.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(STEP4_term.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(STEP4_term.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(STEP4_term.OriX*R2D);
    ui.doubleSpinBox_TargetOriY->setValue(STEP4_term.OriY*R2D);
    ui.doubleSpinBox_TargetOriZ->setValue(STEP4_term.OriZ*R2D);
}
void MainWindow::on_pushButton_Step4_Execute_term_clicked(){
    GuiControlMsg.PosTarget.x = STEP4_term.PosX;
    GuiControlMsg.PosTarget.y = STEP4_term.PosY;
    GuiControlMsg.PosTarget.z = STEP4_term.PosZ;
    GuiControlMsg.OriTarget.x = STEP4_term.OriX;
    GuiControlMsg.OriTarget.y = STEP4_term.OriY;
    GuiControlMsg.OriTarget.z = STEP4_term.OriZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP4_term_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = YPR; //change -> YPR!!!
    GuiControlMsg.InputPosition = GlobalPos;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step4_Execute_term->setText("Running..");
}
//STEP5 End.....
void MainWindow::on_doubleSpinBox_Step5PosX_valueChanged(double arg1){
    STEP5.PosX = arg1;
}
void MainWindow::on_doubleSpinBox_Step5PosY_valueChanged(double arg1){
    STEP5.PosY = arg1;
}
void MainWindow::on_doubleSpinBox_Step5PosZ_valueChanged(double arg1){
    STEP5.PosZ = arg1;
}
void MainWindow::on_doubleSpinBox_Step5OriR_valueChanged(double arg1){
    STEP5.OriX = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step5OriP_valueChanged(double arg1){
    STEP5.OriY = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step5OriY_valueChanged(double arg1){
    STEP5.OriZ = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_Step5Time_valueChanged(double arg1){
    STEP5_Time = arg1;
}

void MainWindow::on_pushButton_Step5_Set_clicked(){
    STEP5.PosX = GuiControlMsg.PosTarget.x;
    STEP5.PosY = GuiControlMsg.PosTarget.y;
    STEP5.PosZ = GuiControlMsg.PosTarget.z;
    STEP5.OriX = GuiControlMsg.OriTarget.x;
    STEP5.OriY = GuiControlMsg.OriTarget.y;
    STEP5.OriZ = GuiControlMsg.OriTarget.z;

    ui.doubleSpinBox_Step5PosX->setValue(STEP5.PosX);
    ui.doubleSpinBox_Step5PosY->setValue(STEP5.PosY);
    ui.doubleSpinBox_Step5PosZ->setValue(STEP5.PosZ);
    ui.doubleSpinBox_Step5OriR->setValue(STEP5.OriX*R2D);
    ui.doubleSpinBox_Step5OriP->setValue(STEP5.OriY*R2D);
    ui.doubleSpinBox_Step5OriY->setValue(STEP5.OriZ*R2D);
}
void MainWindow::on_pushButton_Step5_Get_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(STEP5.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(STEP5.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(STEP5.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(STEP5.OriX*R2D);
    ui.doubleSpinBox_TargetOriY->setValue(STEP5.OriY*R2D);
    ui.doubleSpinBox_TargetOriZ->setValue(STEP5.OriZ*R2D);
}
void MainWindow::on_pushButton_Step5_Execute_clicked(){
    GuiControlMsg.PosTarget.x = STEP5.PosX;
    GuiControlMsg.PosTarget.y = STEP5.PosY;
    GuiControlMsg.PosTarget.z = STEP5.PosZ;
    GuiControlMsg.OriTarget.x = STEP5.OriX;
    GuiControlMsg.OriTarget.y = STEP5.OriY;
    GuiControlMsg.OriTarget.z = STEP5.OriZ;

    MoveFlag = true;
    GuiControlMsg.RunFlag = POS_CTRL;
    GuiControlMsg.EntTime = STEP5_Time;
    GuiControlMsg.UIcmd = RUN;
    GuiControlMsg.InputOrientation = YPR;
    GuiControlMsg.InputPosition = GlobalPos;
    GuiControlPub.publish(GuiControlMsg);
    Current_GuiControlMsg = GuiControlMsg;

    ui.pushButton_Step5_Execute->setText("Running..");
}
//Nostril.....
void MainWindow::on_doubleSpinBox_NostrilPosX_valueChanged(double arg1){
    Nostril.PosX = arg1;
}
void MainWindow::on_doubleSpinBox_NostrilPosY_valueChanged(double arg1){
    Nostril.PosY = arg1;
}
void MainWindow::on_doubleSpinBox_NostrilPosZ_valueChanged(double arg1){
    Nostril.PosZ = arg1;
}
void MainWindow::on_doubleSpinBox_NostrilOriR_valueChanged(double arg1){
    Nostril.OriX = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_NostrilOriP_valueChanged(double arg1){
    Nostril.OriY = arg1*D2R;
}
void MainWindow::on_doubleSpinBox_NostrilOriY_valueChanged(double arg1){
    Nostril.OriZ = arg1*D2R;
}

void MainWindow::on_pushButton_Nostril_Set_clicked(){
    Nostril.PosX = GuiControlMsg.PosTarget.x;
    Nostril.PosY = GuiControlMsg.PosTarget.y;
    Nostril.PosZ = GuiControlMsg.PosTarget.z;
    Nostril.OriX = GuiControlMsg.OriTarget.x;
    Nostril.OriY = GuiControlMsg.OriTarget.y;
    Nostril.OriZ = GuiControlMsg.OriTarget.z;

    ui.doubleSpinBox_NostrilPosX->setValue(Nostril.PosX);
    ui.doubleSpinBox_NostrilPosY->setValue(Nostril.PosY);
    ui.doubleSpinBox_NostrilPosZ->setValue(Nostril.PosZ);
    ui.doubleSpinBox_NostrilOriR->setValue(Nostril.OriX*R2D);
    ui.doubleSpinBox_NostrilOriP->setValue(Nostril.OriY*R2D);
    ui.doubleSpinBox_NostrilOriY->setValue(Nostril.OriZ*R2D);
}
void MainWindow::on_pushButton_Nostril_Get_clicked(){
    ui.doubleSpinBox_TargetPosX->setValue(Nostril.PosX);
    ui.doubleSpinBox_TargetPosY->setValue(Nostril.PosY);
    ui.doubleSpinBox_TargetPosZ->setValue(Nostril.PosZ);
    ui.doubleSpinBox_TargetOriX->setValue(Nostril.OriX*R2D);
    ui.doubleSpinBox_TargetOriY->setValue(Nostril.OriY*R2D);
    ui.doubleSpinBox_TargetOriZ->setValue(Nostril.OriZ*R2D);
}

}  // namespace gui_controller_arm

