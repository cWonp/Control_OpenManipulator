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
#include <iostream>
#include "../include/covid19_communication_client/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace covid19_communication {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(VisualCallbackSignal()), this, SLOT(CallBack_VisualControl_msg()));

    qnode.init();
    cTCP_Packet = new CTCP_Packet;

}

MainWindow::~MainWindow()
{
    delete cTCP_Packet;
    ros::shutdown();
}

void MainWindow::CallBack_VisualControl_msg(){
    std::cout<<"callback"<<std::endl;
    to_arm_msg = qnode.to_arm_msg;

    uint16_t header;
    header = to_arm_msg.RunFlag; //RunFlag 1 : POS ctrl 2: ANG ctrl
    cTCP_Packet->setCommandHeader(header);

    float fData[7];
    if(header == POS_CTRL){
        fData[0] = to_arm_msg.PosX;
        fData[1] = to_arm_msg.PosY;
        fData[2] = to_arm_msg.PosZ;
        fData[3] = to_arm_msg.Yaw;
        fData[4] = to_arm_msg.Pitch;
        fData[5] = to_arm_msg.Roll;
        fData[6] = to_arm_msg.Time;
    }
    else if(header == ANG_CTRL){
        for(int i = 0; i < 6; i++){
            fData[i] = to_arm_msg.JointAng[i];
        }
        fData[6] = to_arm_msg.Time;
    }

    for(int i = 0; i < 7; i++){
        cTCP_Packet->encode(fData[i]);
    }

    cTCP_Packet->sendPacket();

//    //erase!
//    std::cout<<"================================="<<std::endl;
//    std::cout<<"header = "<< header << std::endl;
//    for(int i = 0; i < 7; i++){
//        std::cout<<"fData["<<i<<"] = "<<fData[i]<<std::endl;
//    }
}

void covid19_communication::MainWindow::on_pushButton_Connect_clicked()
{
    cTCP_Packet->connect();
}

void covid19_communication::MainWindow::on_pushButton_Send_Test_clicked()
{
    uint16_t header = 0x0001;
    cTCP_Packet->setCommandHeader(header);

    float fData[7];
    for(int i=0; i<7; i++)
    {
        fData[i] = 1.123 + i;
        cTCP_Packet->encode(fData[i]);
    }

    //    int iData[6];
    //    for(int i=0; i<6; i++)
    //    {
    //        iData[i] = 1 + i;
    //        cTCP_Packet->encode(iData[i]);
    //    }

    cTCP_Packet->sendPacket();
}

} // namespace covid19_communication
