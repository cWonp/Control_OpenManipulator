/**
 * @file /include/covid19_communication/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef covid19_communication_QNODE_HPP_
#define covid19_communication_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <controller_arm_msg/VisualControl.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace covid19_communication {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    ros::Publisher visual_control_pub;//to arm
    controller_arm_msg::VisualControl to_arm_msg;

Q_SIGNALS:
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
};

}  // namespace covid19_communication

#endif /* covid19_communication_QNODE_HPP_ */
