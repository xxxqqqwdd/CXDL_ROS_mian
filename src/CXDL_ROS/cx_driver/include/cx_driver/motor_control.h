
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


#include "controlcan.h"
#include "cx_driver/cxdriver.h"
#include "ros/ros.h"
#include "cx_driver/motor_info.h"
#include "cx_driver/joint_angle_01.h"
#include "cx_driver/joint_angle_08.h"
#include "cx_driver/shengjiang_01.h"
#include "cx_driver/dipan.h"
#include "std_msgs/Bool.h"
#include "cx_driver/feedback.h"



//#ROS收发
ros::Publisher pub_frame_info ;
ros::Publisher pub_motor_feedback;
ros::Subscriber sub_motor_frame_info_08;
ros::Subscriber sub_motor_frame_info_01;

ros::Subscriber sub_shengjiang_info_01;
ros::Subscriber sub_dipan_info;

ros::Subscriber sub_motor_status;




//ROS参数
double CAN_sync_interval;
double CAN_frame_SDO_interval;
double CAN_frame_PDO_interval;


//变量
int m_run0;
int count;
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
double feedback_interval;
//函数
void joint_angle_08_cb(const cx_driver::joint_angle_08::ConstPtr& msg_p);
void joint_angle_01_cb(const cx_driver::joint_angle_01::ConstPtr& msg_p);
void shengjiang_motor_control_01_cb(const cx_driver::shengjiang_01::ConstPtr& msg_p);
void dipan_motor_control_cb(const cx_driver::dipan::ConstPtr& msg_p);

void motor_timer_callback(const ros::TimerEvent);
void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p);
void init(ros::NodeHandle nh);
void initCANConfig(ros::NodeHandle nh);
void initMotorConfig(ros::NodeHandle nh);
void initCANOPENSdo(ros::NodeHandle nh);
void startNMT();
void disableNMT();
void CAN_motor_08(std::vector<double> CAN_joint_Angle);
void CAN_motor_01(std::vector<double>,std::vector<double>);
void CANOPEN_motor_08(std::vector<double> CANOPEN_motor_joint_Angle);
void CANOPEN_motor_01(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>);
void evo_motor_control_08(std::vector<double> evo_angle,std::vector<int> motor_evo_id);
void zhuoyu_motor_control_08(std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id);
void haokong_motor_control_08(std::vector<double> haokong_angle,std::vector<int> motor_haokong_id);
void evo_motor_control_01(std::vector<double> evo_angle,std::vector<int> motor_evo_id);
void zhuoyu_motor_control_01(std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id);
void haokong_motor_control_01(std::vector<double> haokong_angle,std::vector<int> motor_haokong_id);

void motor_status_enable();
void motor_status_disable();
int float_to_uint(float x, float x_min, float x_max, int bits);
void feedback_callback(const ros::TimerEvent);

//结构体
Motor Motor_Vector[15];
Motor CAN_master;

BYTE zhuoyu_sdo[28][8]=
{{0x2F,0x60,0x60,0x00,0x08,0x00,0x00,0x00},
{0x2F,0xC2,0x60,0x01,0x0A,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x80},//2
{0x2F,0x00,0x18,0x02,0xff,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x00,0x1A,0x01,0x20,0x00,0x64,0x60},
{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},
{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x00},//10
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x80},//11
{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x01,0x1A,0x01,0x10,0x00,0x78,0x60},
{0x23,0x01,0x1A,0x02,0x10,0x00,0x41,0x60},
{0x23,0x01,0x1A,0x03,0x10,0x00,0x3F,0x60},
{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x00},//20
{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x80},//21
{0x2F,0x00,0x14,0x02,0x01,0x00,0x00,0x00},
{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},
{0x23,0x00,0x16,0x01,0x20,0x00,0x7a,0x60},
{0x2F,0x00,0x16,0x00,0x01,0x00,0x00,0x00},
{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x00},//26
{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00}
};
BYTE haokong_sdo[29][8]=
{{0x2F,0x60,0x60,0x00,0x08,0x00,0x00,0x00},
{0x2F,0xC2,0x60,0x01,0x0A,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x80},//2
{0x2F,0x00,0x18,0x02,0xff,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x00,0x1A,0x01,0x20,0x00,0x64,0x60},
{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},
{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x00},//10
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x80},//11
{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x01,0x1A,0x01,0x10,0x00,0x78,0x60},
{0x23,0x01,0x1A,0x02,0x10,0x00,0x41,0x60},
{0x23,0x01,0x1A,0x03,0x10,0x00,0x3F,0x60},
{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x00},//20
{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x80},//21
{0x2F,0x00,0x14,0x02,0x01,0x00,0x00,0x00},
{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},
{0x23,0x00,0x16,0x01,0x20,0x00,0x7a,0x60},
{0x2F,0x00,0x16,0x00,0x01,0x00,0x00,0x00},
{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x00},//26
{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00},
{0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00}
};


BYTE shengjiang_sdo[19][8]=
{{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x80},//1
{0x2F,0x00,0x18,0x02,0xff,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x00,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x00,0x1A,0x01,0x20,0x00,0x64,0x60},
{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},
{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},
{0x23,0x00,0x18,0x01,0x00,0x01,0x00,0x00},//9
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x80},//10
{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x03,0x0A,0x00,0x00,0x00},
{0x2b,0x01,0x18,0x05,0x0A,0x00,0x00,0x00},
{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},
{0x23,0x01,0x1A,0x01,0x10,0x00,0x1C,0x22},
{0x23,0x01,0x1A,0x02,0x10,0x00,0x41,0x60},
{0x2F,0x01,0x1A,0x00,0x02,0x00,0x00,0x00},
{0x23,0x01,0x18,0x01,0x00,0x02,0x00,0x00}//18
};


	//evo_enable
	std::vector<BYTE> evo_enable{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

    //evo_disable
	std::vector<BYTE> evo_disable{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

	//disable_id
	std::vector<BYTE> motor_disable{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};

	//ready_id
	std::vector<BYTE> motor_ready{0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};

	//enable_id
	std::vector<BYTE> motor_enable{0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};

    //底盘且模式
	std::vector<BYTE> dipan_enable{0x01};

    //motor_move
    std::vector<BYTE> motor_move{0x2B,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};

    //motor_01_state
    std::vector<BYTE> motor_01_state{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};

    //motor_08_state
    std::vector<BYTE> motor_08_state{0x2F,0x60,0x60,0x00,0x08,0x00,0x00,0x00};


cx_driver::feedback pvfeedback_data;
bool i_X=true;
int vx_i;

bool is_custom_01_v[14]={false,false,false,false,false,false,false,false,false,false,false,false,false,false};
bool is_custom_01_acc[14]={false,false,false,false,false,false,false,false,false,false,false,false,false,false};
bool is_custom_01_dec[14]={false,false,false,false,false,false,false,false,false,false,false,false,false,false};


#endif 
