/**
 * @file /include/gui_controller_arm/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_controller_arm_QNODE_HPP_
#define gui_controller_arm_QNODE_HPP_

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
#include <QStringListModel>
#include <iostream>

#include "controller_arm_msg/StateInfo.h"
#include "controller_arm_msg/GuiControl.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_controller_arm {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
        void run();

        void StateInfoCallback(const controller_arm_msg::StateInfo::ConstPtr& msg);
        controller_arm_msg::StateInfo StateArmInfo;

Q_SIGNALS:
         void rosShutdown();
         void StateInfoSignal();

private:
	int init_argc;
	char** init_argv;
};

}  // namespace gui_controller_arm

#endif /* gui_controller_arm_QNODE_HPP_ */
