/**
 * @file /include/gui_controller_arm/main_window.hpp
 *
 * @brief Qt based gui for gui_controller_arm.
 *
 * @date November 2010
 **/
#ifndef gui_controller_arm_MAIN_WINDOW_H
#define gui_controller_arm_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <iostream>
#include <QString>
#include <fstream>

using namespace std;

/************************
 * Unit
 * Pos [mm]
 * Ang [deg]
 * **********************/

#define PI  acos(-1)
#define R2D 180.0/acos(-1)
#define D2R acos(-1)/180.0

//Run mode
#define STOP                0
#define POS_CTRL            1
#define ANG_CTRL            2
#define MOTION_ROTATING     3
#define STEP_VelocityLimit  4

//GUI CMD
#define RUN             0
#define GetFK           1
#define ChangeInitPos   2
#define ChnageInitAng   3
#define LinearMotion    4
#define VisualCtrlPos   5
#define VisualCtrlAng   6

//InputOrientation
#define YPR         0
#define RCM         1
#define HeadingAng  2

//InputPosition
#define GlobalPos   0
#define LocalPos    1
#define Length      2

//Motion
#define Rotate      0

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_controller_arm {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

extern ros::Publisher GuiControlPub;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    controller_arm_msg::StateInfo StateArmInfo;
    controller_arm_msg::StateInfo StateArmInfo_Init;
    controller_arm_msg::GuiControl GuiControlMsg;
    controller_arm_msg::GuiControl Current_GuiControlMsg;

    bool MoveFlag = false;

    double AngTime = 0;
    double PosTime = 0;

    struct Pose{
        double PosX;
        double PosY;
        double PosZ;
        double OriX;
        double OriY;
        double OriZ;
    };
    struct Pose_LinearMotion
    {
        double length;
        double RCMX;
        double RCMY;
        double RCMZ;
    };
    struct RotateMotion
    {
        double number;
        double angle;
        double time; //for one step
    };

    Pose save1;
    Pose save2;

    //STEP for sampling motion~~~~~~~~~~~~
    Pose STEP1;
    double STEP1_Time = 0;

    Pose STEP2_term;
    double STEP2_term_Time = 0;

    Pose_LinearMotion STEP2;
    double STEP2_Time = 0;

    RotateMotion STEP3;

    Pose_LinearMotion STEP4;
    double STEP4_Time = 0;

    Pose STEP4_term;
    double STEP4_term_Time = 0;

    Pose STEP5;
    double STEP5_Time = 0;

    Pose Nostril;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int POSctrl_InputPos = 0;
    int POSctrl_InputOri = 0;

    uint8_t VelocityLimitmode = 0;

public Q_SLOTS:
    void CallBackStateInfo();

    //Pos order============================================================
    //Target.........
    //Possition
    void on_doubleSpinBox_TargetPosX_valueChanged(double arg1);
    void on_doubleSpinBox_TargetPosY_valueChanged(double arg1);
    void on_doubleSpinBox_TargetPosZ_valueChanged(double arg1);
    void on_doubleSpinBox_TargetPosX_local_valueChanged(double arg1);
    void on_doubleSpinBox_TargetPosY_local_valueChanged(double arg1);
    void on_doubleSpinBox_TargetPosZ_local_valueChanged(double arg1);
    //Orientation
    void on_doubleSpinBox_TargetOriX_valueChanged(double arg1);
    void on_doubleSpinBox_TargetOriY_valueChanged(double arg1);
    void on_doubleSpinBox_TargetOriZ_valueChanged(double arg1);
    void on_radioButton_EnableLocal_clicked(bool checked);

    void on_doubleSpinBox_HeadingAng_Yaw_valueChanged(double arg1);
    void on_doubleSpinBox_HeadingAng_Pitch_valueChanged(double arg1);
    //RCM............
    void on_radioButton_RCMmode_clicked(bool checked);
    void on_radioButton_YPRmode_clicked(bool checked);
    void on_radioButton_HeadingAngmode_clicked(bool checked);
    void on_doubleSpinBox_RCMPosX_valueChanged(double arg1);
    void on_doubleSpinBox_RCMPosY_valueChanged(double arg1);
    void on_doubleSpinBox_RCMPosZ_valueChanged(double arg1);
    //Move...........
    void on_doubleSpinBox_PosControlTime_valueChanged(double arg1);
    void on_pushButton_initialize_pos_clicked();
    void on_pushButton_Set_clicked();
    void on_pushButton_MovePosOrder_clicked();
    void on_radioButton_EnableVelocityLimit_clicked(bool checked);
    //Motion---------------------------------------------------------------
    void on_doubleSpinBox_LinearLength_valueChanged(double arg1);
    void on_pushButton_Set_Linear_clicked();
    //Save............
    void on_pushButton_Set_Save1_clicked();
    void on_pushButton_Get_Save1_clicked();
    void on_pushButton_Set_Save2_clicked();
    void on_pushButton_Get_Save2_clicked();
    //Joint order==========================================================
    void on_doubleSpinBox_JointAng1_valueChanged(double arg1);
    void on_doubleSpinBox_JointAng2_valueChanged(double arg1);
    void on_doubleSpinBox_JointAng3_valueChanged(double arg1);
    void on_doubleSpinBox_JointAng4_valueChanged(double arg1);
    void on_doubleSpinBox_JointAng5_valueChanged(double arg1);
    void on_doubleSpinBox_JointAng6_valueChanged(double arg1);
    void on_doubleSpinBox_JointControlTime_valueChanged(double arg1);
    void on_pushButton_MoveJointControl_clicked();
    void on_pushButton_initialize_ang_clicked();
    //Init=================================================================
    void on_pushButton_MoveInit_clicked();
    void on_pushButton_SetInit_pos_clicked();
    void on_pushButton_SetInit_ang_clicked();
    void on_pushButton_SaveFile_clicked();

    //Sequence control======================================================
    //STEP1 Reach the nostril.....
    void on_doubleSpinBox_Step1PosX_valueChanged(double arg1);
    void on_doubleSpinBox_Step1PosY_valueChanged(double arg1);
    void on_doubleSpinBox_Step1PosZ_valueChanged(double arg1);
    void on_doubleSpinBox_Step1OriR_valueChanged(double arg1);
    void on_doubleSpinBox_Step1OriP_valueChanged(double arg1);
    void on_doubleSpinBox_Step1OriY_valueChanged(double arg1);
    void on_doubleSpinBox_Step1Time_valueChanged(double arg1);

    void on_pushButton_Step1_Set_clicked();
    void on_pushButton_Step1_Get_clicked();
    void on_pushButton_Step1_Execute_clicked();
    //STEP2_term.....
    void on_doubleSpinBox_Step2PosX_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2PosY_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2PosZ_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2OriR_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2OriP_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2OriY_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step2Time_term_valueChanged(double arg1);

    void on_pushButton_Step2_Set_term_clicked();
    void on_pushButton_Step2_Get_term_clicked();
    void on_pushButton_Step2_Execute_term_clicked();
    //STEP2 Insert swab.....
    void on_doubleSpinBox_Step2Length_valueChanged(double arg1);
    void on_doubleSpinBox_Step2RCMX_valueChanged(double arg1);
    void on_doubleSpinBox_Step2RCMY_valueChanged(double arg1);
    void on_doubleSpinBox_Step2RCMZ_valueChanged(double arg1);
    void on_doubleSpinBox_Step2Time_valueChanged(double arg1);

    void on_pushButton_Step2_Set_clicked();
    void on_pushButton_Step2_Get_clicked();
    void on_pushButton_Step2_Execute_clicked();
    //STEP3 Rotate.....
    void on_doubleSpinBox_Step3Number_valueChanged(double arg1);
    void on_doubleSpinBox_Step3Ang_valueChanged(double arg1);
    void on_doubleSpinBox_Step3Time_valueChanged(double arg1);

    void on_pushButton_Step3_Execute_clicked();
    //STEP4 Withdraw swab.....
    void on_doubleSpinBox_Step4Length_valueChanged(double arg1);
    void on_doubleSpinBox_Step4RCMX_valueChanged(double arg1);
    void on_doubleSpinBox_Step4RCMY_valueChanged(double arg1);
    void on_doubleSpinBox_Step4RCMZ_valueChanged(double arg1);
    void on_doubleSpinBox_Step4Time_valueChanged(double arg1);

    void on_pushButton_Step4_Set_clicked();
    void on_pushButton_Step4_Get_clicked();
    void on_pushButton_Step4_Execute_clicked();
    //STEP4_term.....
    void on_doubleSpinBox_Step4PosX_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4PosY_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4PosZ_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4OriR_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4OriP_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4OriY_term_valueChanged(double arg1);
    void on_doubleSpinBox_Step4Time_term_valueChanged(double arg1);

    void on_pushButton_Step4_Set_term_clicked();
    void on_pushButton_Step4_Get_term_clicked();
    void on_pushButton_Step4_Execute_term_clicked();
    //STEP5 End.....
    void on_doubleSpinBox_Step5PosX_valueChanged(double arg1);
    void on_doubleSpinBox_Step5PosY_valueChanged(double arg1);
    void on_doubleSpinBox_Step5PosZ_valueChanged(double arg1);
    void on_doubleSpinBox_Step5OriR_valueChanged(double arg1);
    void on_doubleSpinBox_Step5OriP_valueChanged(double arg1);
    void on_doubleSpinBox_Step5OriY_valueChanged(double arg1);
    void on_doubleSpinBox_Step5Time_valueChanged(double arg1);

    void on_pushButton_Step5_Set_clicked();
    void on_pushButton_Step5_Get_clicked();
    void on_pushButton_Step5_Execute_clicked();
    //Nostril.....
    void on_doubleSpinBox_NostrilPosX_valueChanged(double arg1);
    void on_doubleSpinBox_NostrilPosY_valueChanged(double arg1);
    void on_doubleSpinBox_NostrilPosZ_valueChanged(double arg1);
    void on_doubleSpinBox_NostrilOriR_valueChanged(double arg1);
    void on_doubleSpinBox_NostrilOriP_valueChanged(double arg1);
    void on_doubleSpinBox_NostrilOriY_valueChanged(double arg1);

    void on_pushButton_Nostril_Set_clicked();
    void on_pushButton_Nostril_Get_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace gui_controller_arm

#endif // gui_controller_arm_MAIN_WINDOW_H
