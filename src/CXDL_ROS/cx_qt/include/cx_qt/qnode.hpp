
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
#include "../../../devel/include/cx_driver/joint_angle.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/



/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
//    explicit QNode(QObject *parent = nullptr);

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
    void moter_ceshi_launch_slot(cx_driver::joint_angle angles);

    //电机使能
    void on_btn_motorenable_clicked_slot();

    //电机失能
    void on_btn_motordisable_clicked_slot();




signals:

    // 定义一个信号，当接收到电机反馈时发射

    void motorFeedbackReceived(cx_driver::feedback*);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();














private:


	int init_argc;
	char** init_argv;
    QStringListModel logging_model;
    //电机反馈
    ros::Subscriber sub_motor_feedback_info ;
    //发送角度
    ros::Publisher pub_angle_info;
    //电机使能
    ros::Publisher sub_motor_status;


    //电机反馈_用于传递参数
    cx_driver::feedback* msg;

    cx_driver::joint_angle angles;
};



#endif /* CX_QT_QNODE_HPP_ */
