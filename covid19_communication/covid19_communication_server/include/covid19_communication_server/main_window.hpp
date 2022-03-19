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
#include <QtNetwork>
#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>
#include <QObject>
#include <vector>

//Run mode
#define STOP        0
#define POS_CTRL    1
#define ANG_CTRL    2

#define STEP_PC_TCPIP_PORT 2222

#define VISION_PC_IP "192.168.0.101"
//#define STEP_PC_IP "192.168.0.101"
#define STEP_PC_IP "192.168.0.100"

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024
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
    void newConnection();
    void disconnected();
    void readyRead();
    void decodeData(QByteArray Ddata);
    void sendToArm();

Q_SIGNALS:
    void dataReceived(QByteArray Rdata);
    void dataDecoded();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;

    QTcpServer *tcpServer;
    QHash<QTcpSocket*, QByteArray*> buffers; //We need a buffer to store data until block has completely received
    QHash<QTcpSocket*, qint32*> sizes; //We need to store the size to verify if a block has received completely

    uint16_t header;
    float to_arm[7];
    bool to_arm_done;
};

}  // namespace covid19_communication

#endif // covid19_communication_MAIN_WINDOW_H
