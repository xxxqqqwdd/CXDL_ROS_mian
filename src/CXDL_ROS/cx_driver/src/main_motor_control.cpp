#include <iostream>
#include "ros/ros.h"
#include "cx_driver/controlcan.h"
#include "cx_driver/cxdriver.h"
#include "cx_driver/motor_control.h"
#include "cx_driver/motor_info.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <unistd.h>



#define BYTE unsigned char



int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"motor_driver");
    ros::NodeHandle nh;


	//程序初始化
	init(nh);
	//初始化CAN卡
    initCANConfig(nh);
	//初始化电机
	initMotorConfig(nh);
	//启动NMT
	startNMT();
	//初始化SDO/PDO
    initCANOPENSdo(nh);

	
    ros::Timer sync_interval_Timer = nh.createTimer(ros::Duration(CAN_sync_interval/1000), motor_timer_callback);
	ros::Timer feedback_Timer = nh.createTimer(ros::Duration(feedback_interval/1000), feedback_callback);

    pub_frame_info= nh.advertise<cx_driver::motor_info>("motor_frame_info",10);
	pub_motor_feedback= nh.advertise<cx_driver::feedback>("motor_feedback",10);
	


    sub_motor_frame_info_08 = nh.subscribe<cx_driver::joint_angle_08>("joint_angle_08",10,joint_angle_08_cb);
	sub_motor_frame_info_01 = nh.subscribe<cx_driver::joint_angle_01>("joint_angle_01",10,joint_angle_01_cb);

	sub_shengjiang_info_01 = nh.subscribe<cx_driver::shengjiang_01>("shengjiang_01",10,shengjiang_motor_control_01_cb);
	sub_dipan_info = nh.subscribe<cx_driver::dipan>("dipan",10,dipan_motor_control_cb);

	sub_motor_status = nh.subscribe<std_msgs::Bool>("motor_status",10,motor_status_cb);

   
    m_run0=1;
	pthread_t threadid;
	int ret;
	void *receive_func(void* param);
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
	
    ros::spin();
	m_run0=0;
	disableNMT();

	usleep(10000);
    return 0;
}





