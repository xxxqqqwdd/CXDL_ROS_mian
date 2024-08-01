
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


#include "../../../devel/include/cx_driver/feedback.h"


class mainwindows;


/*****************************************************************************
** Namespaces
*****************************************************************************/



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



    void motor_feedback_cb(const cx_driver::feedback::ConstPtr& msg_p);



public slots:
    //my
    void ros_launch_start();


signals:

    // 定义一个信号，当接收到电机反馈时发射

    void motorFeedbackReceived(const cx_driver::feedbackConstPtr& msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();














private:
	int init_argc;
	char** init_argv;
    QStringListModel logging_model;
    ros::Subscriber sub_motor_feedback_info ;
};



#endif /* CX_QT_QNODE_HPP_ */
