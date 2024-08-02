/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cx_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {

    msg =new cx_driver::feedback;

    ros::init(init_argc,init_argv,"cx_qt_node");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;


    sub_motor_feedback_info=nh.subscribe<cx_driver::feedback>("/motor_feedback",10,&QNode::motor_feedback_cb,this);
    pub_angle_info= nh.advertise<cx_driver::joint_angle>("joint_angle",10);
    start();

	return true;
}



void QNode::motor_feedback_cb(const cx_driver::feedback::ConstPtr& msg_p)

{

    // 当接收到电机反馈时，发射信号

    msg->stamp=msg_p->stamp;
    msg->position=msg_p->position;
    msg->velocity=msg_p->velocity;
    msg->torque=msg_p->torque;

    emit motorFeedbackReceived(msg);

}

//启动电机
void QNode::ros_launch_start()
{
    system("gnome-terminal -x bash -c 'source /home/u/CXDL/code/git/CXDL_ROS_main/devel/setup.bash; roslaunch cx_driver cx_driver.launch'&");
}

void QNode::moter_ceshi_(int a)
{


    ros::Rate loop_rate(100);

    for (int i=0;i<3141;i++){
        double angle_1=sin(i/1000.0)*M_PI / 3;
        angles.left_arm_joint[a-1]= angle_1;
        pub_angle_info.publish(angles);
        loop_rate.sleep();
        std::cout<<"tttttttttttttttttt"<<std::endl;
    }

}



void QNode::run() {



ros::spin();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}




