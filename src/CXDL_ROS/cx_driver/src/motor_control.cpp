#include "cx_driver/cxdriver.h"
#include "ros/ros.h"
#include "cx_driver/motor_info.h"
#include "cx_driver/controlcan.h"
#include "cx_driver/motor_control.h"
#include "cx_driver/joint_angle.h"
#include "std_msgs/Bool.h"




void joint_angle_cb(const cx_driver::joint_angle::ConstPtr& msg_p){
	std::vector<double> CAN_joint_Angle{msg_p->left_arm_joint.at(4),
										msg_p->right_arm_joint.at(4)};
	CAN_motor(CAN_joint_Angle);
	std::vector<double> CANOPEN_joint_Angle{msg_p->left_arm_joint.at(0),
									 	msg_p->left_arm_joint.at(1),
										msg_p->left_arm_joint.at(2),
										msg_p->left_arm_joint.at(3),
										msg_p->left_arm_joint.at(5),
										msg_p->right_arm_joint.at(0),
										msg_p->right_arm_joint.at(1),
										msg_p->right_arm_joint.at(2),
										msg_p->right_arm_joint.at(3),
										msg_p->right_arm_joint.at(5)};
	CANOPEN_motor(CANOPEN_joint_Angle);
}

void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p){
	if(msg_p->data == 1)
       motor_status_enable();
    else
       motor_status_disable();
}

void motor_status_enable()
{
	//evo_enable
	std::vector<BYTE> evo_enable{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

	//disable_id
	std::vector<BYTE> zhuoyu_haokong_disable{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
	//ready_id
	std::vector<BYTE> haokong_ready{0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};
	//enable_id
	std::vector<BYTE> zhuoyu_haokong_enable{0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};


	for(int v_m=0;v_m<Motor_Vector.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			for(int j=0;j<=evo_enable.size();j++)
			{
				Motor_Vector.at(v_m).get()->setData(j,evo_enable.at(j));
			}
			Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

		}
		else
		{
			if(v_m == 0||v_m == 1||v_m == 2||v_m == 6||v_m == 7||v_m == 8)
			{
				for(int j=0;j<=zhuoyu_haokong_disable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_haokong_disable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

				for(int j=0;j<=zhuoyu_haokong_enable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_haokong_enable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

			}
			else
			{
				for(int j=0;j<=zhuoyu_haokong_disable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_haokong_disable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

				for(int j=0;j<=haokong_ready.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,haokong_ready.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

				for(int j=0;j<=zhuoyu_haokong_enable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_haokong_enable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

			}
		}
	}	
}
void motor_status_disable()
{
	//evo_disable
	std::vector<BYTE> evo_enable{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

	//disable_id
	std::vector<BYTE> zhuoyu_haokong_disable{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};

	for(int v_m=0;v_m<evo_enable.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			for(int j=0;j<=evo_enable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,evo_enable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);

		}
		else
		{
			for(int j=0;j<=zhuoyu_haokong_disable.size();j++)
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_haokong_disable.at(j));
				}
				Motor_Vector.at(v_m).get()->transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);
		}
	}		
}

	



void motor_timer_callback(const ros::TimerEvent){
	
if(is_sync_ok == TRUE)
{
	CAN_master.setSendType(0);
	CAN_master.setRemoteFlag(0);
	CAN_master.setExternFlag(0);
	CAN_master.setSdoID(0X80);
	CAN_master.setDataLen(0);
	CAN_master.transmit(CAN_frame_SDO_interval,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);

	int all_pdo_storage_longth = all_pdo_storage.size();
	int evo_sdo_storage_longth = evo_sdo_storage.size();
	
	
	VCI_CAN_OBJ evo_sdo_send[evo_sdo_storage_longth];

	VCI_CAN_OBJ pdo_send[all_pdo_storage_longth];


	for(int i= 0;i<evo_sdo_storage_longth;i++)
	{
		evo_sdo_send[i].ID=evo_sdo_storage_id.at(i);
		evo_sdo_send[i].SendType=0;
		evo_sdo_send[i].RemoteFlag=0;
		evo_sdo_send[i].ExternFlag=0;
		evo_sdo_send[i].DataLen=8;

		for(int j=0;j<=evo_sdo_storage.at(i).size();j++)
		{
			evo_sdo_send[i].Data[j]=evo_sdo_storage.at(i).at(j);
		}
	}
	transmit(VCI_USBCAN2, 0, 0, evo_sdo_send, evo_sdo_storage_longth);



	for(int i= 0;i<all_pdo_storage_longth;i++)
	{
		pdo_send[i].ID=all_pdo_storage_id.at(i);
		pdo_send[i].SendType=0;
		pdo_send[i].RemoteFlag=0;
		pdo_send[i].ExternFlag=0;
		pdo_send[i].DataLen=4;

		for(int j=0;j<=all_pdo_storage.at(i).size();j++)
		{
			pdo_send[i].Data[j]=all_pdo_storage.at(i).at(j);
		}
	}
	transmit(VCI_USBCAN2, 0, 0, pdo_send, all_pdo_storage_longth);

	is_sync_ok = false;
	all_pdo_storage.clear();
	all_pdo_storage_id.clear();
	evo_sdo_storage.clear();
	evo_sdo_storage_id.clear();

	is_evo_sdo_ok = false;
	is_zhuoyu_pdo_ok = false;
	is_haokong_pdo_ok = false;
	

}
}

void init(ros::NodeHandle nh)
{
	is_evo_sdo_ok=false;
	is_zhuoyu_pdo_ok=false;
	is_haokong_pdo_ok=false;
	is_sync_ok =false;
	nh.param("motor/evo/acceleration",evo_acceleration);
	nh.param("motor/evo/P_MIN",P_MIN);
	nh.param("motor/evo/P_MAX",P_MAX);
	nh.param("motor/evo/V_MIN",V_MIN);
	nh.param("motor/evo/V_MAX",V_MAX);
	nh.param("motor/evo/P_KP_MIN",P_KP_MIN);
	nh.param("motor/evo/P_KP_MAX",P_KP_MAX);
	nh.param("motor/evo/P_KD_MIN",P_KD_MIN);
	nh.param("motor/evo/P_KD_MAX",P_KD_MAX);
	nh.param("motor/evo/V_KP_MIN",V_KP_MIN);
	nh.param("motor/evo/V_KP_MAX",V_KP_MAX);
	nh.param("motor/evo/V_KD_MIN",V_KD_MIN);
	nh.param("motor/evo/V_KD_MAX",V_KD_MAX);
	nh.param("motor/evo/V_KI_MAX",V_KI_MAX);
	nh.param("motor/evo/V_KI_MIN",V_KI_MIN);

	p_kp0 = 15;
	p_kd0 = 4.5;
	v_kp0 = 50;
	v_kd0 = 0;
	v_ki0 = 0.001;

	all_pdo_storage.clear();
	all_pdo_storage_id.clear();
	evo_sdo_storage.clear();
	evo_sdo_storage_id.clear();
}


void initCANConfig(ros::NodeHandle nh){
	
	VCI_BOARD_INFO pInfo;//用来获取设备信息。
	VCI_BOARD_INFO pInfo1 [50];
	int num=0;
	num=VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		
		printf(">>open deivce success!\n");//打开设备成功

		
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

   //初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;
	config.Timing1=0x14;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}


}

void initMotorConfig(ros::NodeHandle nh){



Motor_Vector.push_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x601,0x201,20));  
Motor_Vector.push_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x602,0x202,20));  
Motor_Vector.push_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x603,0x203,20));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::HAOKONG,0x604,0x204,16));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::EVO,0x005,0x005));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::HAOKONG,0x606,0x206,16));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x607,0x207,20)); 
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x608,0x208,20));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::ZHUOYU,0x609,0x209,20));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::HAOKONG,0x60A,0x20A,16));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::EVO,0x00B,0x00B));  
// Motor_Vector.emplace_back(std::make_unique<Motor>(MotorType::HAOKONG,0x60C,0x20C,16));  


//CAN
nh.getParam("/CAN/frame_SDO_interval",CAN_frame_SDO_interval);
nh.getParam("/CAN/frame_PDO_interval",CAN_frame_PDO_interval);
nh.getParam("/CAN/sync_interval",CAN_sync_interval);

std::cout<<"xxxx"<<CAN_frame_SDO_interval<<std::endl;
std::cout<<"xxxx"<<CAN_sync_interval<<std::endl;
}


void initCANOPENSdo(ros::NodeHandle nh){

for(int v_m=0;v_m<Motor_Vector.size();v_m++){
if(v_m == 4||v_m == 10)
{
continue;
}
Motor_Vector.at(v_m).get()->setSendType(0);
Motor_Vector.at(v_m).get()->setRemoteFlag(0);
Motor_Vector.at(v_m).get()->setExternFlag(0);
Motor_Vector.at(v_m).get()->setDataLen(8);
}

//循环位置模式帧
std::vector<BYTE> mode_set{0x2F,0x60,0x60,0x00,0x08,0x00,0x00,0x00};//0
zhuoyu_sdo.push_back(mode_set);
haokong_sdo.push_back(mode_set);
//电机运行间隔
std::vector<BYTE> moteo_interval{0x2F,0xC2,0x60,0x01,0x1E,0x00,0x00,0x00};
zhuoyu_sdo.push_back(moteo_interval);
haokong_sdo.push_back(moteo_interval);
//tpdo1_1
std::vector<BYTE> tpdo1_1{0x23,0x00,0x18,0x01,0x00,0x18,0x00,0x80};//2
zhuoyu_sdo.push_back(tpdo1_1);
haokong_sdo.push_back(tpdo1_1);
//tpdo1_2
std::vector<BYTE> tpdo1_2{0x2F,0x00,0x18,0x02,0xfe,0x00,0x00,0x00};
zhuoyu_sdo.push_back(tpdo1_2);
haokong_sdo.push_back(tpdo1_2);
//tpdo1_3
std::vector<BYTE> tpdo1_3{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00};
zhuoyu_sdo.push_back(tpdo1_3);
haokong_sdo.push_back(tpdo1_3);
//tpdo1_4
std::vector<BYTE> tpdo1_4{0x23,0x00,0x1A,0x01,0x20,0x00,0x64,0x60};
zhuoyu_sdo.push_back(tpdo1_4);
haokong_sdo.push_back(tpdo1_4);
//tpdo1_5
std::vector<BYTE> tpdo1_5{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60};
zhuoyu_sdo.push_back(tpdo1_5);
haokong_sdo.push_back(tpdo1_5);
//tpdo1_6
std::vector<BYTE> tpdo1_6{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00};
zhuoyu_sdo.push_back(tpdo1_6);
haokong_sdo.push_back(tpdo1_6);
//tpdo1_7
std::vector<BYTE> tpdo1_7{0x23,0x00,0x18,0x01,0x00,0x18,0x00,0x00};//8
zhuoyu_sdo.push_back(tpdo1_7);
haokong_sdo.push_back(tpdo1_7);
//tpdo2_1
std::vector<BYTE> tpdo2_1{0x23,0x01,0x18,0x01,0x00,0x28,0x00,0x80};//9
zhuoyu_sdo.push_back(tpdo2_1);
haokong_sdo.push_back(tpdo2_1);
//tpdo2_2
std::vector<BYTE> tpdo2_2{0x2F,0x01,0x18,0x02,0xFE,0x00,0x00,0x80};
zhuoyu_sdo.push_back(tpdo2_2);
haokong_sdo.push_back(tpdo2_2);
//tpdo2_3
std::vector<BYTE> tpdo2_3{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00};
zhuoyu_sdo.push_back(tpdo2_3);
haokong_sdo.push_back(tpdo2_3);
//tpdo2_4
std::vector<BYTE> tpdo2_4{0x23,0x01,0x1A,0x01,0x10,0x00,0x77,0x60};
zhuoyu_sdo.push_back(tpdo2_4);
haokong_sdo.push_back(tpdo2_4);
//tpdo2_5
std::vector<BYTE> tpdo2_5{0x23,0x01,0x1A,0x02,0x10,0x00,0x41,0x60};
zhuoyu_sdo.push_back(tpdo2_5);
haokong_sdo.push_back(tpdo2_5);
//tpdo2_6
std::vector<BYTE> tpdo2_6{0x23,0x01,0x1A,0x03,0x10,0x00,0x3F,0x60};
zhuoyu_sdo.push_back(tpdo2_6);
haokong_sdo.push_back(tpdo2_6);
//tpdo2_7
std::vector<BYTE> tpdo2_7{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00};
zhuoyu_sdo.push_back(tpdo2_7);
haokong_sdo.push_back(tpdo2_7);
//tpdo2_8
std::vector<BYTE> tpdo2_8{0x23,0x01,0x18,0x01,0x00,0x28,0x00,0x00};//16
zhuoyu_sdo.push_back(tpdo2_8);
haokong_sdo.push_back(tpdo2_8);
//rpdo1_1
std::vector<BYTE> rpdo1_1{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x80};//17
zhuoyu_sdo.push_back(rpdo1_1);
haokong_sdo.push_back(rpdo1_1);
//rpdo1_2
std::vector<BYTE> rpdo1_2{0x23,0x00,0x14,0x02,0x01,0x00,0x00,0x00};
zhuoyu_sdo.push_back(rpdo1_2);
haokong_sdo.push_back(rpdo1_2);
//rpdo1_3
std::vector<BYTE> rpdo1_3{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00};
zhuoyu_sdo.push_back(rpdo1_3);
haokong_sdo.push_back(rpdo1_3);
//rpdo1_4
std::vector<BYTE> rpdo1_4{0x23,0x00,0x16,0x01,0x20,0x00,0x7a,0x60};
zhuoyu_sdo.push_back(rpdo1_4);
haokong_sdo.push_back(rpdo1_4);
//rpdo1_5
std::vector<BYTE> rpdo1_5{0x2F,0x00,0x16,0x00,0x01,0x00,0x00,0x00};
zhuoyu_sdo.push_back(rpdo1_5);
haokong_sdo.push_back(rpdo1_5);
//rpdo1_6
std::vector<BYTE> rpdo1_6{0x23,0x00,0x14,0x01,0x00,0x02,0x00,0x00};//22
zhuoyu_sdo.push_back(rpdo1_6);
haokong_sdo.push_back(rpdo1_6);

//disable_id
std::vector<BYTE> disable_id{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
zhuoyu_sdo.push_back(disable_id);
haokong_sdo.push_back(disable_id);
//ready_id
std::vector<BYTE> ready_id{0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};
haokong_sdo.push_back(ready_id);
//enable_id
std::vector<BYTE> enable_id{0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};
zhuoyu_sdo.push_back(enable_id);
haokong_sdo.push_back(enable_id);

for(int v_m = 0;v_m<Motor_Vector.size();v_m++)
{
	if(v_m == 0||v_m == 1||v_m == 2||v_m == 6||v_m == 7||v_m == 8)
	{
		for(int sdo_row_i=0;sdo_row_i<zhuoyu_sdo.size();sdo_row_i++){
			for(int j=0;j<zhuoyu_sdo.at(sdo_row_i).size();j++)
			{
				if(sdo_row_i==2||sdo_row_i==8||sdo_row_i==9||sdo_row_i==16||sdo_row_i==17||sdo_row_i==22)
				{
				zhuoyu_sdo.at(sdo_row_i).at(5)=(v_m+1);
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_sdo.at(sdo_row_i).at(j));
				}
				else
				{
				Motor_Vector.at(v_m).get()->setData(j,zhuoyu_sdo.at(sdo_row_i).at(j));
								
			}
			}
			
		Motor_Vector.at(v_m).get()->setID(Motor_Vector.at(v_m).get()->getSdoID());
		// Motor_Vector.at(v_m).get()->transmit(VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);
		
		// CAN_frame_SDO_interval*1000,
		std::cout<<ii<<std::endl;
		ii++;
		}
	}

if(v_m == 3||v_m == 5||v_m == 9||v_m == 11)
{
for(int sdo_row_i=0;sdo_row_i<haokong_sdo.size();sdo_row_i++){
for(int j=0;j<haokong_sdo.at(sdo_row_i).size();j++)
{
if(sdo_row_i==2||sdo_row_i==8||sdo_row_i==9||sdo_row_i==16||sdo_row_i==17||sdo_row_i==22)
{
haokong_sdo.at(sdo_row_i).at(5)=(v_m+1);
Motor_Vector.at(v_m).get()->setData(j,haokong_sdo.at(sdo_row_i).at(j));
}
else
{
Motor_Vector.at(v_m).get()->setData(j,haokong_sdo.at(sdo_row_i).at(j));
}
}
Motor_Vector.at(v_m).get()->setID(Motor_Vector.at(v_m).get()->getSdoID());
// Motor_Vector.at(v_m).get()->transmit(VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).get()->getFrame(), 1);
// CAN_frame_SDO_interval*1000,
std::cout<<ii<<std::endl;
ii++;
}
}
}
ROS_INFO("pdo配置完成!!!");

}





void startNMT()
{
	CAN_master.setSdoID(0X000);
	CAN_master.setSendType(0);
	CAN_master.setRemoteFlag(0);
	CAN_master.setExternFlag(0);
	CAN_master.setDataLen(2);

	std::vector<BYTE> can_sdo{0x01,0x00};
	for(int j=0;j<can_sdo.size();j++)
	{
		CAN_master.setData(j,can_sdo.at(j));
	}
	CAN_master.transmit(CAN_frame_SDO_interval,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);
}




void CAN_motor(std::vector<double> CAN_joint_Angle)
{
	std::vector<int> motor_evo_id{5,11};
	evo_motor_control(CAN_joint_Angle,motor_evo_id);
}

void CANOPEN_motor(std::vector<double> CANOPEN_motor_joint_Angle)
{
	std::vector<double> zhuoyu_motor{CANOPEN_motor_joint_Angle.at(0),
									CANOPEN_motor_joint_Angle.at(1),
									CANOPEN_motor_joint_Angle.at(2),
									CANOPEN_motor_joint_Angle.at(5),
									CANOPEN_motor_joint_Angle.at(6),
									CANOPEN_motor_joint_Angle.at(7)};
	std::vector<int> motor_zhuoyu_id{1,2,3,7,8,9};
	zhuoyu_motor_control(zhuoyu_motor,motor_zhuoyu_id);
	std::vector<double> haokong_motor{CANOPEN_motor_joint_Angle.at(3),
									CANOPEN_motor_joint_Angle.at(4),
									CANOPEN_motor_joint_Angle.at(8),
									CANOPEN_motor_joint_Angle.at(9),};
	std::vector<int> motor_haokong_id{4,6,10,12};
	haokong_motor_control(haokong_motor,motor_haokong_id);
}





int double_to_unit(double x, double x_min, double x_max, int bits)
{

double span = x_max - x_min;
double offset = x_min;
return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
}

void evo_motor_control(std::vector<double> evo_angle,std::vector<int> motor_evo_id)
{
	// while(1)
	{
		if(!is_sync_ok)
		{
			for(int i_1=0;i_1<motor_evo_id.size();i_1++)
			{
			double v_i = 0.5*evo_acceleration*(CAN_sync_interval-sqrt(CAN_sync_interval*CAN_sync_interval-4*evo_angle.at(i_1)/evo_acceleration));

			Motor_Vector.at(motor_evo_id.at(i_1)-1).get()->setID(Motor_Vector.at(motor_evo_id.at(i_1)-1).get()->getPdoID());

			int P0 = double_to_unit(evo_angle.at(i_1), P_MIN, P_MAX, 16);
			int V0 = double_to_unit(v_i, V_MIN, V_MAX, 8);
			int P_kp0 = double_to_unit(p_kp0, P_KP_MIN, P_KP_MAX, 8);
			int P_kd0 = double_to_unit(p_kd0, P_KD_MIN, P_KD_MAX, 8);
			int V_kp0 = double_to_unit(v_kp0,V_KP_MIN,V_KP_MAX,8);
			int V_kd0 = double_to_unit(v_kd0,V_KD_MIN,V_KD_MAX,8);
			int V_ki0 = double_to_unit(v_ki0,V_KI_MIN,V_KI_MAX,8);

			BYTE d1= P0 >> 8;
			BYTE d2= P0 & 0xFF;
			BYTE d3= V0;
			BYTE d4= P_kp0;
			BYTE d5=P_kd0;
			BYTE d6=V_kp0;
			BYTE d7=V_kd0;
			BYTE d8=V_ki0;

			std::vector<BYTE> single_evo_sdo_storage{d1,d2,d3,d4,d5,d6,d7,d8};
			evo_sdo_storage.push_back(single_evo_sdo_storage);
			evo_sdo_storage_id.push_back(motor_evo_id.at(i_1));

			}
			is_evo_sdo_ok = true;
			if(is_evo_sdo_ok||is_zhuoyu_pdo_ok||is_haokong_pdo_ok)
			{
				is_sync_ok = true;
			}

			// break;
		}
	}
}

void zhuoyu_motor_control(std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id)
{
	// while(1)
	{
		if(!is_sync_ok)
		{
			for(int i_1 = 0;i_1<zhuoyu_angle.size();i_1++){
				_Float32 dpi = pow(2, Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).get()->getEncodingRate())/ (2*M_PI);
				int result = dpi *zhuoyu_angle.at(i_1);
				BYTE d1=(result&0xff);
				BYTE d2=(result>>8);
				BYTE d3=(result>>16);
				BYTE d4=(result>>24);
			std::vector<BYTE> single_pdo_storage{d1,d2,d3,d4};

			all_pdo_storage.push_back(single_pdo_storage);
			all_pdo_storage_id.push_back(motor_zhuoyu_id.at(i_1));
				}
		is_zhuoyu_pdo_ok = true;
		if(is_evo_sdo_ok||is_zhuoyu_pdo_ok||is_haokong_pdo_ok)
			{
				is_sync_ok = true;
			}

			// break;
	}
}
}

void haokong_motor_control(std::vector<double> haokong_angle,std::vector<int> motor_haokong_id)
{
	// while(1)
	{
		if(!is_sync_ok)
			{
			for(int i_1 = 0;i_1<haokong_angle.size();i_1++){
				_Float32 dpi = pow(2, Motor_Vector.at(motor_haokong_id.at(i_1)-1).get()->getEncodingRate())/ (2*M_PI);
				int result = dpi *haokong_angle.at(i_1);
				BYTE d1=(result&0xff);
				BYTE d2=(result>>8);
				BYTE d3=(result>>16);
				BYTE d4=(result>>24);
			std::vector<BYTE> single_pdo_storage{d1,d2,d3,d4};

			all_pdo_storage.push_back(single_pdo_storage);
			all_pdo_storage_id.push_back(motor_haokong_id.at(i_1));
				}
			}

		is_haokong_pdo_ok=true;
		if(is_evo_sdo_ok||is_zhuoyu_pdo_ok||is_haokong_pdo_ok)
			{
				is_sync_ok = true;
			}

			// break;
	}

}












