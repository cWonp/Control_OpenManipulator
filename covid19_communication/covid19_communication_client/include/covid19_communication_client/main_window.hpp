/**
 * @file /include/covid19_communication/main_window.hpp
 *
 * @brief Qt based gui for covid19_communication.
 *
 * @date November 2010
 **/
#ifndef covid19_communication_MAIN_WINDOW_H
#define covid19_communication_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "ctcp_packet.h"
#include "controller_arm_msg/VisualControl.h"
#include <iostream>

//Run mode
#define STOP        0
#define POS_CTRL    1
#define ANG_CTRL    2

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace covid19_communication {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void CallBack_VisualControl_msg();

    void on_pushButton_Connect_clicked();
    void on_pushButton_Send_Test_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    CTCP_Packet* cTCP_Packet;

    controller_arm_msg::VisualControl to_arm_msg;
};

}  // namespace covid19_communication

#endif // covid19_communication_MAIN_WINDOW_H
