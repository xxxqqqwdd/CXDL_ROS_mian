
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include "controlcan.h"
#include "cx_driver/cxdriver.h"
#include "ros/ros.h"
#include "cx_driver/motor_info.h"
#include "cx_driver/joint_angle.h"
#include "std_msgs/Bool.h"



//#ROS收发
ros::Publisher pub_frame_info ;
ros::Subscriber sub_frame_info;
ros::Subscriber sub_motor_status;




//ROS参数
double CAN_sync_interval;
double CAN_frame_SDO_interval;
double CAN_frame_PDO_interval;


//变量
int count;
BOOL is_sync_ok;
BOOL is_zhuoyu_pdo_ok;
BOOL is_haokong_pdo_ok;

//函数
void joint_angle_cb(const cx_driver::joint_angle::ConstPtr& msg_p);
void motor_timer_callback(const ros::TimerEvent);
void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p);
void initCANConfig(ros::NodeHandle nh);
void initMotorConfig(ros::NodeHandle nh);
void initCANOPENSdo(ros::NodeHandle nh);
void startNMT();
void CAN_motor(double time,std::vector<double> CAN_joint_Angle);
void CANOPEN_motor(double time,std::vector<double> CANOPEN_motor_joint_Angle);
void evo_motor_control(double time,std::vector<double> evo_angle,std::vector<int> motor_evo_id);
void zhuoyu_motor_control(double time ,std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id);
void haokong_motor_control(double time ,std::vector<double> haokong_angle,std::vector<int> motor_haokong_id);
void motor_status_enable();
void motor_status_disable();
std::vector<int> section(double time ,int value);


//结构体
Motor motor_evo_1;
Motor motor_evo_2;
Motor motor_zhuoyu_1;
Motor motor_zhuoyu_2;
Motor motor_zhuoyu_3;
Motor motor_zhuoyu_4;
Motor motor_zhuoyu_5;
Motor motor_zhuoyu_6;
Motor motor_haokong_1;
Motor motor_haokong_2;
Motor motor_haokong_3;
Motor motor_haokong_4;
Motor CAN_master;

std::vector<Motor> Motor_Vector;


std::vector<std::vector<BYTE>> zhuoyu_sdo;
std::vector<std::vector<BYTE>> haokong_sdo;

std::vector<std::vector<std::vector<BYTE>>> all_pdo_storage;
/////////////////////
// 
// 




// void pdo_transmit(double time , double joint);




#endif 