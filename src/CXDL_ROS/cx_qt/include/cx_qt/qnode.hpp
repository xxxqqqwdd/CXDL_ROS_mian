
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef CX_QT_QNODE_HPP_
#define CX_QT_QNODE_HPP_

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
//#include "main_window.hpp"
#include "ui_main_window.h"
#include "../../../devel/include/cx_driver/feedback.h"


class mainwindows;


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace CX_QT {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:

//  explicit QNode(UI::MainWindow *p);
//  UI::MainWindow* exUI;



	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();



  ros::Subscriber sub_frame_info ;
  void motor_feedback_cb(const cx_driver::feedback &msg);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace CX_QT

#endif /* CX_QT_QNODE_HPP_ */
