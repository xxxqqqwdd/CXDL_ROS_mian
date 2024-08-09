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
#include "std_msgs/Bool.h"
#include <QThreadPool>
#include <QDebug>


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

    is_joint_inited=false;

    msg =new cx_driver::feedback;

    ros::init(init_argc,init_argv,"cx_qt_node");

    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    start();

    return true;
}



void QNode::motor_feedback_cb(const cx_driver::feedback::ConstPtr& msg_p)

{


    // if(!is_joint_inited){
    //     for(int i =0 ;i<14;i++){
    //         if(0<=i&&i<6)
    //         angles.left_arm_joint.at(i)=msg_p->position[i];
    //         if(6<=i&&i<12)
    //         angles.right_arm_joint.at(i-6)=msg_p->position[i];
    //         if(12<=i&&i<14)
    //         angles.shengjiang.at(i-12)=msg_p->position[i];
    //     }
    //     is_joint_inited = true;
    //     }

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


    //电机发送
    void QNode::moter_ceshi_launch_slot(cx_driver::joint_angle_08 angles)
    {
        pub_angle_info.publish(angles);
    }

    //电机使能
    void QNode::on_btn_motorenable_clicked_slot()
    {
        std_msgs::Bool motor_status_true;
        motor_status_true.data=true;
        pub_motor_status.publish(motor_status_true);
    }

    //电机失能
    void QNode::on_btn_motordisable_clicked_slot()
    {
        std_msgs::Bool motor_status_false;
        motor_status_false.data=false;
        pub_motor_status.publish(motor_status_false);
    }


    //控制升降
    void QNode::on_btn_set_H_clicked_slot(double h)
    {
        h=(h-155-85)/2;
        double s = 505.5937-abs(sqrt(255625-75000*cos(M_PI/2-asin(h/500))));

        //TODO:当前的行程需要换算成角度
    }



    void QNode::run() {

        ros::NodeHandle nh;

        //电机反馈
        sub_motor_feedback_info=nh.subscribe<cx_driver::feedback>("/motor_feedback",10,&QNode::motor_feedback_cb,this);

        //发送角度
        pub_angle_info= nh.advertise<cx_driver::joint_angle_08>("/joint_angle_08",10);

        //电机使能
        pub_motor_status = nh.advertise<std_msgs::Bool>("/motor_status",10);



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




