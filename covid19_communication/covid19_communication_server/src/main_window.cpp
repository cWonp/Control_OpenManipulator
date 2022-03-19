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
#include "../include/covid19_communication_server/main_window.hpp"

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
    qnode.init();

    QHostAddress VISION_PC_Address;
    VISION_PC_Address.setAddress(VISION_PC_IP);

    tcpServer = new QTcpServer();

    //if(!tcpServer->listen(VISION_PC_Address, STEP_PC_TCPIP_PORT))
    if(!tcpServer->listen(QHostAddress::Any, STEP_PC_TCPIP_PORT))
    {
        //qDebug() << "Server could not start!";
    }
    else
    {
        //qDebug() << "Server started!";
    }

    header = 0;
    to_arm_done = true;
    for(int i=0; i<7; i++)
    {
        to_arm[i]=0;
    }

    QObject::connect(tcpServer, SIGNAL(newConnection()), this, SLOT(newConnection()));
    QObject::connect(this, SIGNAL(dataReceived(QByteArray)), this, SLOT(decodeData(QByteArray)));
    QObject::connect(this, SIGNAL(dataDecoded()), this, SLOT(sendToArm()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
    delete tcpServer;
    ros::shutdown();
}

void MainWindow::newConnection()
{
    qDebug() << "CONNECTED";
    ui.label_data->setText("connected!");
    while (tcpServer->hasPendingConnections())
    {
        //qDebug() << "while start";
        QTcpSocket *socket = tcpServer->nextPendingConnection();
        QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
        QObject::connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));
        QByteArray *buffer = new QByteArray();
        qint32 *s = new qint32(0);
        buffers.insert(socket, buffer);
        sizes.insert(socket, s);
    }
}

qint32 ArrayToInt(QByteArray source)
{
    qint32 temp;
    QDataStream data(&source, QIODevice::ReadWrite);
    data >> temp;
    return temp;
}

void MainWindow::readyRead()
{
    //qDebug() << "readyRead";
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    QByteArray *buffer = buffers.value(socket);
    qint32 *s = sizes.value(socket);
    qint32 size = *s;

    while (socket->bytesAvailable() > 0)
    {
        buffer->append(socket->readAll());
        while ((size == 0 && buffer->size() >= 4) || (size > 0 && buffer->size() >= size)) //While can process data, process it
        {
            if (size == 0 && buffer->size() >= 4) //if size of data has received completely, then store it on our global variable
            {
                size = ArrayToInt(buffer->mid(0, 4));
                *s = size;
                buffer->remove(0, 4);
            }

            if (size > 0 && buffer->size() >= size) // If data has received completely, then Q_EMIT our SIGNAL with the data
            {
                //ui->textBrowser->insertPlainText(QString(buffer->data()));
                QByteArray data = buffer->mid(0, size);
                buffer->remove(0, size);
                size = 0;
                *s = size;
                Q_EMIT dataReceived(data);
            }
        }
    }
}

void MainWindow::decodeData(QByteArray Ddata)
{
    if(to_arm_done)
    {
        //qDebug() << "decodeData";
        QString result;

        if(Ddata.at(0)==13 && Ddata.at(1)==10)
        {
            header = Ddata.at(2);
            for(int i=1; i<8; i++)
            {
                float now_num;
                char now_data[4] = {Ddata.at(i*4),Ddata.at(i*4+1),Ddata.at(i*4+2),Ddata.at(i*4+3)};
                memcpy(&now_num, now_data, sizeof(now_data));
                to_arm[i-1] = now_num;
                result.append("Data["+QString::number(i-1)+"] : " +QString::number(to_arm[i-1])+"\r\n");
            }

            to_arm_done = false;
            Q_EMIT dataDecoded();
            ui.label_data->setText(result);
        }
    }
}

void MainWindow::sendToArm()
{
    qnode.to_arm_msg.RunFlag = header;

    if(header==POS_CTRL)
    {
        qnode.to_arm_msg.PosX = to_arm[0];
        qnode.to_arm_msg.PosY = to_arm[1];
        qnode.to_arm_msg.PosZ = to_arm[2];
        qnode.to_arm_msg.Yaw = to_arm[3];
        qnode.to_arm_msg.Pitch = to_arm[4];
        qnode.to_arm_msg.Roll = to_arm[5];
        qnode.to_arm_msg.Time = to_arm[6];
    }
    else if(header==ANG_CTRL)
    {
        qnode.to_arm_msg.JointAng[0] = to_arm[0];
        qnode.to_arm_msg.JointAng[1] = to_arm[1];
        qnode.to_arm_msg.JointAng[2] = to_arm[2];
        qnode.to_arm_msg.JointAng[3] = to_arm[3];
        qnode.to_arm_msg.JointAng[4] = to_arm[4];
        qnode.to_arm_msg.JointAng[5] = to_arm[5];
        qnode.to_arm_msg.JointAng[6] = to_arm[6];
    }
    qnode.visual_control_pub.publish(qnode.to_arm_msg);
    to_arm_done = true;

    //qDebug() << " !!!!! To Arm !!!!! ";
}

void MainWindow::disconnected()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    QByteArray *buffer = buffers.value(socket);
    qint32 *s = sizes.value(socket);
    socket->deleteLater();
    delete buffer;
    delete s;
}

} // namespace covid19_communication
