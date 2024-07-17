
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
BOOL is_evo_sdo_ok;
BOOL is_zhuoyu_pdo_ok;
BOOL is_haokong_pdo_ok;
double evo_acceleration;
double P_MIN;
double P_MAX;
double V_MIN;
double V_MAX;
double P_KP_MIN;
double P_KP_MAX;
double P_KD_MIN;
double P_KD_MAX;
double V_KP_MIN;
double V_KP_MAX;
double V_KD_MIN;
double V_KD_MAX;
double V_KI_MAX;
double V_KI_MIN;
double p_kp0;
double p_kd0;
double v_kp0;
double v_kd0;
double v_ki0;

//函数
void joint_angle_cb(const cx_driver::joint_angle::ConstPtr& msg_p);
void motor_timer_callback(const ros::TimerEvent);
void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p);
void init(ros::NodeHandle nh);
void initCANConfig(ros::NodeHandle nh);
void initMotorConfig(ros::NodeHandle nh);
void initCANOPENSdo(ros::NodeHandle nh);
void startNMT();
void CAN_motor(std::vector<double> CAN_joint_Angle);
void CANOPEN_motor(std::vector<double> CANOPEN_motor_joint_Angle);
void evo_motor_control(std::vector<double> evo_angle,std::vector<int> motor_evo_id);
void zhuoyu_motor_control(std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id);
void haokong_motor_control(std::vector<double> haokong_angle,std::vector<int> motor_haokong_id);
void motor_status_enable();
void motor_status_disable();



//结构体

std::vector<Motor> Motor_Vector ;
Motor CAN_master ;

std::vector<std::vector<BYTE>> zhuoyu_sdo;
std::vector<std::vector<BYTE>> haokong_sdo;

std::vector<std::vector<BYTE>> all_pdo_storage;
std::vector<BYTE> all_pdo_storage_id;

std::vector<std::vector<BYTE>> evo_sdo_storage;
std::vector<BYTE> evo_sdo_storage_id;
/////////////////////



#endif 