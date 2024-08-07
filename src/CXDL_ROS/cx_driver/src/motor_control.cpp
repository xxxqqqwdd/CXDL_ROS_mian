#include "cx_driver/cxdriver.h"
#include "ros/ros.h"
#include "cx_driver/motor_info.h"
#include "cx_driver/controlcan.h"
#include "cx_driver/motor_control.h"
#include "cx_driver/joint_angle.h"
#include "std_msgs/Bool.h"

#include <mutex> 


std::mutex mtx; 


void *receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j,motor_i;
	double zhuoyu_ItoT,haokong_ItoT,shengjiang_ItoT=0.06;
	int32_t zhuoyu_P,zhuoyu_V,haokong_P,haokong_V,shengjiang_P,shengjiang_V,dipan_mileage_left,dipan_mileage_right;
    int16_t zhuoyu_T,haokong_T,shengjiang_T,dipan_move_v,dipan_rotate_v;  
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				std::lock_guard<std::mutex> lock(mtx);  

				if (rec[j].ID==5||rec[j].ID==11)  //motorevo
				{
					motor_i = rec[j].ID-1;
                    pvfeedback_data.position[motor_i]=((rec[j].Data[1]*pow(2,8)+rec[j].Data[2])*25/pow(2,16))-12.5;
					pvfeedback_data.velocity[motor_i]=((rec[j].Data[3]*pow(2,4)+int(rec[j].Data[4])/int(16))*20/pow(2,12))-10;
					pvfeedback_data.torque[motor_i]=(((int(rec[j].Data[4])%int(16))*pow(2,8)+rec[j].Data[5])*100/pow(2,12))-50;
					continue;
				}
				if (rec[j].ID==385||rec[j].ID==386||rec[j].ID==387||rec[j].ID==391||rec[j].ID==392||rec[j].ID==393)   //zhuoyu
				{
					motor_i = rec[j].ID-385;
					zhuoyu_P = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8) | (rec[j].Data[2] << 16) | (rec[j].Data[3] << 24);
					zhuoyu_V = (rec[j].Data[4] << 0) | (rec[j].Data[5] << 8) | (rec[j].Data[6] << 16) | (rec[j].Data[7] << 24);					
                    pvfeedback_data.position[motor_i]=zhuoyu_P*(2*M_PI)/pow(2,20);
					pvfeedback_data.velocity[motor_i]=zhuoyu_V*(2*M_PI)/pow(2,20);  //28c58 1rad/s
					continue;
				}
				if (rec[j].ID==641||rec[j].ID==642||rec[j].ID==643||rec[j].ID==647||rec[j].ID==648||rec[j].ID==649)   //zhuoyu
				{
					
					motor_i = rec[j].ID-641;
					zhuoyu_T = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8);
					pvfeedback_data.torque[motor_i]=zhuoyu_T/1000*zhuoyu_ItoT;
					continue;
				}
				if (rec[j].ID==388||rec[j].ID==390||rec[j].ID==394||rec[j].ID==396)   //haokong
				{
					motor_i = rec[j].ID-385;
					haokong_P = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8) | (rec[j].Data[2] << 16) | (rec[j].Data[3] << 24);
					haokong_V = (rec[j].Data[4] << 0) | (rec[j].Data[5] << 8) | (rec[j].Data[6] << 16) | (rec[j].Data[7] << 24);		
                    pvfeedback_data.position[motor_i]=haokong_P*(2*M_PI)/pow(2,16);
					pvfeedback_data.velocity[motor_i]=haokong_V*10*(2*M_PI)/60;
					continue;
				}
				if (rec[j].ID==644||rec[j].ID==646||rec[j].ID==650||rec[j].ID==652)   //haokong
				{
					motor_i = rec[j].ID-641;
					haokong_T = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8);
					pvfeedback_data.torque[motor_i]=haokong_T/100*haokong_ItoT;
					continue;
				}
				if (rec[j].ID==397||rec[j].ID==398)   //升降
				{
					motor_i = rec[j].ID-385;
					shengjiang_P = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8) | (rec[j].Data[2] << 16) | (rec[j].Data[3] << 24);
					shengjiang_V = (rec[j].Data[4] << 0) | (rec[j].Data[5] << 8) | (rec[j].Data[6] << 16) | (rec[j].Data[7] << 24);		
                    pvfeedback_data.position[motor_i]=shengjiang_P/131072*0.25;   //mm   H=sin(0.5*M_PI-arccos(（255625-pow((505.5937-(shengjiang_P/131072*0.25)）,2)）/75000))*500
					pvfeedback_data.velocity[motor_i]=shengjiang_V/21845333*1000*0.25/60;  //mm/s
					continue;
				}
				if (rec[j].ID==653||rec[j].ID==654)   //升降
				{
					motor_i = rec[j].ID-641;
					shengjiang_T = (rec[j].Data[0] << 0) | (rec[j].Data[1] << 8);
					pvfeedback_data.torque[motor_i]=shengjiang_T/1000*shengjiang_ItoT;
					continue;
				}
				if (rec[j].ID==545)   //底盘
				{
					motor_i = 14;
					dipan_move_v = (rec[j].Data[0] << 8) | (rec[j].Data[1] << 0);     //mm/s
					dipan_rotate_v = (rec[j].Data[2] << 8) | (rec[j].Data[3] << 0)/1000;   //rad/s
					pvfeedback_data.velocity[motor_i]=dipan_move_v;
					pvfeedback_data.velocity[motor_i+1]=dipan_rotate_v;
					continue;
				}
				if (rec[j].ID==785)   //底盘
				{
					dipan_mileage_left = (rec[j].Data[0] << 24) | (rec[j].Data[1] << 16) | (rec[j].Data[2] << 8) | (rec[j].Data[3] << 0);     //mm
					dipan_mileage_right = (rec[j].Data[4] << 24) | (rec[j].Data[5] << 16) | (rec[j].Data[6] << 8) | (rec[j].Data[7] << 0);   
					pvfeedback_data.mileage[0]=dipan_mileage_left;
					pvfeedback_data.mileage[1]=dipan_mileage_right;
					continue;
					}
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
			}

		}
		ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}





void feedback_callback(const ros::TimerEvent){

	std::lock_guard<std::mutex> lock(mtx);
	pvfeedback_data.stamp = ros::Time::now();
	pub_motor_feedback.publish(pvfeedback_data);

}






void joint_angle_cb(const cx_driver::joint_angle::ConstPtr& msg_p){
	std::vector<double> CAN_joint_Angle{msg_p->left_arm_joint.at(4),
										msg_p->right_arm_joint.at(4),
										msg_p->dipan.at(0),
										msg_p->dipan.at(1)
										};
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
										msg_p->right_arm_joint.at(5),
										msg_p->shengjiang.at(0),
									 	msg_p->shengjiang.at(1)};

	CANOPEN_motor(CANOPEN_joint_Angle);
	
}

void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p){
	if(msg_p->data == true)
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
	std::vector<BYTE> dipan_enable{0x01};

	for(int v_m=0;v_m<12;v_m++){

		Motor_Vector[v_m].setID(Motor_Vector[v_m].getSdoID());
		Motor_Vector[v_m].setSendType(0);
		Motor_Vector[v_m].setRemoteFlag(0);
		Motor_Vector[v_m].setExternFlag(0);
		Motor_Vector[v_m].setDataLen(8);


		if(v_m == 4||v_m == 10)
		{
			for(int j=0;j<evo_enable.size();j++)
			{
				Motor_Vector[v_m].setData(j,evo_enable.at(j));
			}
			Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

		}
		else
		{
			if(v_m == 0||v_m == 1||v_m == 2||v_m == 6||v_m == 7||v_m == 8)
			{
				for(int j=0;j<zhuoyu_haokong_disable.size();j++)
				{
				Motor_Vector[v_m].setData(j,zhuoyu_haokong_disable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

				for(int j=0;j<zhuoyu_haokong_enable.size();j++)
				{
				Motor_Vector[v_m].setData(j,zhuoyu_haokong_enable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

			}
			else
			{
				for(int j=0;j<zhuoyu_haokong_disable.size();j++)
				{
				Motor_Vector[v_m].setData(j,zhuoyu_haokong_disable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

				for(int j=0;j<haokong_ready.size();j++)
				{
				Motor_Vector[v_m].setData(j,haokong_ready.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

				for(int j=0;j<zhuoyu_haokong_enable.size();j++)
				{
				Motor_Vector[v_m].setData(j,zhuoyu_haokong_enable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

			}
		}
	}
	Motor_Vector[14].setID(Motor_Vector[14].getPdoID());
	Motor_Vector[14].setSendType(0);
	Motor_Vector[14].setRemoteFlag(0);
	Motor_Vector[14].setExternFlag(0);
	Motor_Vector[14].setDataLen(1);
	Motor_Vector[14].setData(0,dipan_enable.at(0));
    Motor_Vector[14].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[14].getFrame(), 1);

}
void motor_status_disable()
{
	//evo_disable
	std::vector<BYTE> evo_disable{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

	//disable_id
	std::vector<BYTE> zhuoyu_haokong_disable{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};

	std::vector<BYTE> shengjiang_disable{0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00};

	for(int v_m=0;v_m<14;v_m++){

		Motor_Vector[v_m].setID(Motor_Vector[v_m].getSdoID());
		Motor_Vector[v_m].setSendType(0);
		Motor_Vector[v_m].setRemoteFlag(0);
		Motor_Vector[v_m].setExternFlag(0);
		Motor_Vector[v_m].setDataLen(8);


		if(v_m == 4||v_m == 10)
		{

				for(int j=0;j<evo_disable.size();j++)
				{
				Motor_Vector[v_m].setData(j,evo_disable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

		}
		else
			if(v_m == 12||v_m == 13)
			{

				for(int j=0;j<shengjiang_disable.size();j++)
				{
				Motor_Vector[v_m].setData(j,shengjiang_disable.at(j));
				}
				Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);

			}
			else
			{
				for(int j=0;j<zhuoyu_haokong_disable.size();j++)
					{
					Motor_Vector[v_m].setData(j,zhuoyu_haokong_disable.at(j));
					}
					Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);
			}
	}		

}

	



void motor_timer_callback(const ros::TimerEvent){
	

	CAN_master.setSendType(0);
	CAN_master.setRemoteFlag(0);
	CAN_master.setExternFlag(0);
	CAN_master.setSdoID(0X80);
	CAN_master.setDataLen(0);
	CAN_master.setID(CAN_master.getSdoID());
	CAN_master.transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);

}

void init(ros::NodeHandle nh)
{
	nh.getParam("/motor/evo/acceleration",evo_acceleration);
	nh.getParam("/motor/evo/P_MIN",P_MIN);
	nh.getParam("/motor/evo/P_MAX",P_MAX);
	nh.getParam("/motor/evo/V_MIN",V_MIN);
	nh.getParam("/motor/evo/V_MAX",V_MAX);
	nh.getParam("/motor/evo/P_KP_MIN",P_KP_MIN);
	nh.getParam("/motor/evo/P_KP_MAX",P_KP_MAX);
	nh.getParam("/motor/evo/P_KD_MIN",P_KD_MIN);
	nh.getParam("/motor/evo/P_KD_MAX",P_KD_MAX);
	nh.getParam("/motor/evo/V_KP_MIN",V_KP_MIN);
	nh.getParam("/motor/evo/V_KP_MAX",V_KP_MAX);
	nh.getParam("/motor/evo/V_KD_MIN",V_KD_MIN);
	nh.getParam("/motor/evo/V_KD_MAX",V_KD_MAX);
	nh.getParam("/motor/evo/V_KI_MAX",V_KI_MAX);
	nh.getParam("/motor/evo/V_KI_MIN",V_KI_MIN);
	nh.getParam("/QT/feedback_interval",feedback_interval);
	p_kp0 = 15;
	p_kd0 = 4.5;
	v_kp0 = 50;
	v_kd0 = 0;
	v_ki0 = 0.001;


	vx_i = 0;
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

Motor zhuoyu_1(MotorType::ZHUOYU,0x601,0x201,20);
Motor_Vector[0]=zhuoyu_1;
Motor zhuoyu_2(MotorType::ZHUOYU,0x602,0x202,20);
Motor_Vector[1]=zhuoyu_2;
Motor zhuoyu_3(MotorType::ZHUOYU,0x603,0x203,20);
Motor_Vector[2]=zhuoyu_3;
Motor hangkong_1(MotorType::HAOKONG,0x604,0x204,16);
Motor_Vector[3]=hangkong_1;
Motor evo_1(MotorType::EVO,0x005);

Motor_Vector[4]=evo_1;
Motor hangkong_2(MotorType::HAOKONG,0x606,0x206,16);
Motor_Vector[5]=hangkong_2;

Motor zhuoyu_4(MotorType::ZHUOYU,0x607,0x207,20);
Motor_Vector[6]=zhuoyu_4;
Motor zhuoyu_5(MotorType::ZHUOYU,0x608,0x208,20);
Motor_Vector[7]=zhuoyu_5;
Motor zhuoyu_6(MotorType::ZHUOYU,0x609,0x209,20);
Motor_Vector[8]=zhuoyu_6;
Motor hangkong_3(MotorType::HAOKONG,0x60A,0x20A,16);
Motor_Vector[9]=hangkong_3;
Motor evo_2(MotorType::EVO,0x00B);
Motor_Vector[10]=evo_2;
Motor hangkong_4(MotorType::HAOKONG,0x60C,0x20C,16);
Motor_Vector[11]=hangkong_4;

Motor shengjiang_1(MotorType::SHENGJIANG,0x60D,0x20D);  
Motor shengjiang_2(MotorType::SHENGJIANG,0x60E,0x20E);
Motor_Vector[12]=shengjiang_1;
Motor_Vector[13]=shengjiang_2;

Motor dipan(MotorType::DIPAN,0x111,0x421);
Motor_Vector[14]=dipan;


//CAN
nh.getParam("/CAN/frame_SDO_interval",CAN_frame_SDO_interval);
nh.getParam("/CAN/frame_PDO_interval",CAN_frame_PDO_interval);
nh.getParam("/CAN/sync_interval",CAN_sync_interval);

// std::cout<<"/CAN/sync_interval"<<CAN_sync_interval<<std::endl;
}


void initCANOPENSdo(ros::NodeHandle nh){

for(int v_m=0;v_m<14;v_m++){      //底盘不配

Motor_Vector[v_m].setID(Motor_Vector[v_m].getSdoID());
Motor_Vector[v_m].setSendType(0);
Motor_Vector[v_m].setRemoteFlag(0);
Motor_Vector[v_m].setExternFlag(0);
Motor_Vector[v_m].setDataLen(8);
}

int zhuoyu_size = sizeof(zhuoyu_sdo)/sizeof(zhuoyu_sdo[0]);
int haokong_size = sizeof(haokong_sdo)/sizeof(haokong_sdo[0]);
int shengjiang_size = sizeof(shengjiang_sdo)/sizeof(shengjiang_sdo[0]);

for(int v_m = 0;v_m<14;v_m++)
{
	if(v_m == 0||v_m == 1||v_m == 2||v_m == 6||v_m == 7||v_m == 8)
	{
		for(int sdo_row_i=0;sdo_row_i<zhuoyu_size;sdo_row_i++){
			for(int j=0;j<8;j++)
			{
				if(sdo_row_i==2||sdo_row_i==10||sdo_row_i==11||sdo_row_i==20)
				{	
				zhuoyu_sdo[sdo_row_i][4]=(v_m+129);
				Motor_Vector[v_m].setData(j,zhuoyu_sdo[sdo_row_i][j]);
				}
				else if(sdo_row_i==21||sdo_row_i==26)
				{
				zhuoyu_sdo[sdo_row_i][4]=(v_m+1);
				Motor_Vector[v_m].setData(j,zhuoyu_sdo[sdo_row_i][j]);}
				else
				{Motor_Vector[v_m].setData(j,zhuoyu_sdo[sdo_row_i][j]);
								
			}
			}
			
		
		Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);
		// std::cout<<Motor_Vector[v_m].getID()<<std::endl;
		// for(int iii=0;iii<8;iii++)
		// {
		// 	int aa=Motor_Vector[v_m].getData(iii);
		// 	// std::cout<<aa<<" ";
		// }
//		std::cout<<std::endl;
		// std::cout<<Motor_Vector[v_m].getFrame()<<std::endl;
		}
	}

	if(v_m == 3||v_m == 5||v_m == 9||v_m == 11)
	{
		for(int sdo_row_i=0;sdo_row_i<haokong_size;sdo_row_i++){
			for(int j=0;j<8;j++)
			{
			if(sdo_row_i==2||sdo_row_i==10||sdo_row_i==11||sdo_row_i==20)
			{
			haokong_sdo[sdo_row_i][4]=(v_m+129);
			Motor_Vector[v_m].setData(j,haokong_sdo[sdo_row_i][j]);
			}
			else if(sdo_row_i==21||sdo_row_i==26)
			{
			haokong_sdo[sdo_row_i][4]=(v_m+1);
			Motor_Vector[v_m].setData(j,haokong_sdo[sdo_row_i][j]);}
			else
			{
			Motor_Vector[v_m].setData(j,haokong_sdo[sdo_row_i][j]);
			}
			}

		Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);
		}
	}
	if(v_m == 12||v_m == 13)
	{
		for(int sdo_row_i=0;sdo_row_i<shengjiang_size;sdo_row_i++){
			for(int j=0;j<8;j++)
			{
				if(sdo_row_i==2||sdo_row_i==10||sdo_row_i==11||sdo_row_i==20)
			{
			shengjiang_sdo[sdo_row_i][4]=(v_m+129);
			Motor_Vector[v_m].setData(j,shengjiang_sdo[sdo_row_i][j]);
			}
			else{
			Motor_Vector[v_m].setData(j,shengjiang_sdo[sdo_row_i][j]);
			}
			}

		Motor_Vector[v_m].transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[v_m].getFrame(), 1);
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
	CAN_master.setID(CAN_master.getSdoID());
	std::vector<BYTE> can_sdo{0x01,0x00};
	for(int j=0;j<can_sdo.size();j++)
	{
		CAN_master.setData(j,can_sdo.at(j));
	}
	CAN_master.transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);
}


void disableNMT()
{
	CAN_master.setSdoID(0X000);
	CAN_master.setSendType(0);
	CAN_master.setRemoteFlag(0);
	CAN_master.setExternFlag(0);
	CAN_master.setDataLen(2);
	CAN_master.setID(CAN_master.getSdoID());
	std::vector<BYTE> can_sdo{0x02,0x00};
	for(int j=0;j<can_sdo.size();j++)
	{
		CAN_master.setData(j,can_sdo.at(j));
	}
	CAN_master.transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);
}




void CAN_motor(std::vector<double> CAN_joint_Angle)
{
	std::vector<double> evo_motor{CAN_joint_Angle.at(0),
								CAN_joint_Angle.at(1)};
	std::vector<int> motor_evo_id{5,11};
	evo_motor_control(evo_motor,motor_evo_id);
	std::vector<double> dipan_motor{CAN_joint_Angle.at(2),
									CAN_joint_Angle.at(3)};
	std::vector<int> motor_dipan_id{15};
	dipan_motor_control(dipan_motor,motor_dipan_id);

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
									CANOPEN_motor_joint_Angle.at(9)};
	std::vector<int> motor_haokong_id{4,6,10,12};
	haokong_motor_control(haokong_motor,motor_haokong_id);
	std::vector<double> shengjiang_motor{CANOPEN_motor_joint_Angle.at(10),
									CANOPEN_motor_joint_Angle.at(11)};
	std::vector<int> motor_shengjiang_id{13,14};
	shengjiang_motor_control(shengjiang_motor,motor_shengjiang_id);




}





int float_to_uint(float x, float x_min, float x_max, int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void shengjiang_motor_control(std::vector<double> shengjiang_angle,std::vector<int> motor_shengjiang_id)
{

		for(int i_1 = 0;i_1<shengjiang_angle.size();i_1++){

		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setID(Motor_Vector[motor_shengjiang_id.at(i_1)-1].getSdoID());
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setSendType(0);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setRemoteFlag(0);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setExternFlag(0);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setDataLen(8);
		for(int j=0;j<8;j++)
		{
			Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(j,shengjiang_touch[0][j]);
			}
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_shengjiang_id.at(i_1)-1].getFrame(), 1);	

		int result = shengjiang_angle.at(i_1)/(2*M_PI)*131072;
		BYTE d1=(0x23);
		BYTE d2=(0x7A);
		BYTE d3=(0x60);
		BYTE d4=(0x00);
		BYTE d5=(result&0xff);
		BYTE d6=(result>>8);
		BYTE d7=(result>>16);
		BYTE d8=(result>>24);

		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(0,d1);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(1,d2);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(2,d3);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(3,d4);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(4,d5);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(5,d6);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(6,d7);
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(7,d8);

		Motor_Vector[motor_shengjiang_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_shengjiang_id.at(i_1)-1].getFrame(), 1);		
		for(int j=0;j<8;j++)
		{
			Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(j,shengjiang_v[0][j]);
			}
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_shengjiang_id.at(i_1)-1].getFrame(), 1);	
		for(int j=0;j<8;j++)
		{
			Motor_Vector[motor_shengjiang_id.at(i_1)-1].setData(j,shengjiang_move[0][j]);
			}
		Motor_Vector[motor_shengjiang_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_shengjiang_id.at(i_1)-1].getFrame(), 1);	
			}
		
}
void dipan_motor_control(std::vector<double> dipan_v,std::vector<int> dipan_id)
{
	Motor_Vector[dipan_id.at(0)-1].setID(Motor_Vector[dipan_id.at(0)-1].getSdoID());
	Motor_Vector[dipan_id.at(0)-1].setSendType(0);
	Motor_Vector[dipan_id.at(0)-1].setRemoteFlag(0);
	Motor_Vector[dipan_id.at(0)-1].setExternFlag(0);
	Motor_Vector[dipan_id.at(0)-1].setDataLen(8);
	BYTE d1=(int(dipan_v.at(0))>>8);
	BYTE d2=(int(dipan_v.at(0))&0xff);
	BYTE d3=(int(dipan_v.at(1))>>8);
	BYTE d4=(int(dipan_v.at(1))&0xff);
	BYTE d5=(0x00);
	BYTE d6=(0x00);
	BYTE d7=(0x00);
	BYTE d8=(0x00);
	Motor_Vector[dipan_id.at(0)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[dipan_id.at(0)-1].getFrame(), 1);
}



void evo_motor_control(std::vector<double> evo_angle,std::vector<int> motor_evo_id)
{
			vx_i++;
			for(int i_1=0;i_1<motor_evo_id.size();i_1++)
			{
			// double v_i = 0.5*evo_acceleration*((CAN_sync_interval/1000)-sqrt((CAN_sync_interval/1000)*(CAN_sync_interval/1000)-((4*evo_angle.at(i_1))/evo_acceleration)));

			//  for (int ii=2;ii<800;ii++){
        		double v_i=cos(vx_i/300.0)*2*M_PI/9*1.5;
				
				std::cout<<"v_i"<<v_i<<std::endl;
			//  }

			
			int P0 =float_to_uint(evo_angle.at(i_1), P_MIN, P_MAX, 16);
			int V0 = float_to_uint(abs(v_i), V_MIN, V_MAX, 8);
			int P_kp0 = float_to_uint(p_kp0, P_KP_MIN, P_KP_MAX, 8);
			int P_kd0 = float_to_uint(p_kd0, P_KD_MIN, P_KD_MAX, 8);
			int V_kp0 = float_to_uint(v_kp0,V_KP_MIN,V_KP_MAX,8);
			int V_kd0 = float_to_uint(v_kd0,V_KD_MIN,V_KD_MAX,8);
			int V_ki0 = float_to_uint(v_ki0,V_KI_MIN,V_KI_MAX,8);
			

			int aa;
			// std::cout<<P0<<"  "<<V0<<" "<<aa<<std::endl;
			// printf("P0 %d \n", P0);
			// printf("V0 %d", V0);

			BYTE d1= P0 >> 8;
			BYTE d2= P0 & 0xFF;
			BYTE d3= V0;
			BYTE d4= P_kp0;
			BYTE d5=P_kd0;
			BYTE d6=V_kp0;
			BYTE d7=V_kd0;
			BYTE d8=V_ki0;

			
			Motor_Vector[motor_evo_id.at(i_1)-1].setID(Motor_Vector[motor_evo_id.at(i_1)-1].getSdoID());
			Motor_Vector[motor_evo_id.at(i_1)-1].setSendType(0);
			Motor_Vector[motor_evo_id.at(i_1)-1].setRemoteFlag(0);
			Motor_Vector[motor_evo_id.at(i_1)-1].setExternFlag(0);
			Motor_Vector[motor_evo_id.at(i_1)-1].setDataLen(8);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(0,d1);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(1,d2);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(2,d3);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(3,d4);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(4,d5);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(5,d6);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(6,d7);
			Motor_Vector[motor_evo_id.at(i_1)-1].setData(7,d8);

			printf("vx_i,%d",vx_i);
			printf("\n");
			for(int i = 0; i <8; i++)
				{
					printf(" %02X", Motor_Vector[motor_evo_id.at(i_1)-1].getData(i));
				}
			printf("\n");
			Motor_Vector[motor_evo_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_evo_id.at(i_1)-1].getFrame(), 1);		
			printf("***************");
			}


}

void zhuoyu_motor_control(std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id)
{

			for(int i_1 = 0;i_1<zhuoyu_angle.size();i_1++){
				
				_Float32 dpi = pow(2, Motor_Vector[motor_zhuoyu_id.at(i_1)-1].getEncodingRate())/ (2*M_PI);
				int result = dpi *zhuoyu_angle.at(i_1);
				BYTE d1=(result&0xff);
				BYTE d2=(result>>8);
				BYTE d3=(result>>16);
				BYTE d4=(result>>24);
			

			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setID(Motor_Vector[motor_zhuoyu_id.at(i_1)-1].getPdoID());
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setSendType(0);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setRemoteFlag(0);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setExternFlag(0);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setDataLen(4);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setData(0,d1);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setData(1,d2);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setData(2,d3);
			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].setData(3,d4);
			// std::cout<<Motor_Vector[motor_zhuoyu_id.at(i_1)-1].getPdoID()<<std::endl;

			// if(i_X){
			// for(int i = 0; i <4; i++)
			// 	{
			// 		printf(" %02X", Motor_Vector[motor_zhuoyu_id.at(i_1)-1].getData(i));
			// 	}
			// 	printf("\n");
			// 	// i_X=false;
			// }

			Motor_Vector[motor_zhuoyu_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_zhuoyu_id.at(i_1)-1].getFrame(), 1);		
			

				}

}

void haokong_motor_control(std::vector<double> haokong_angle,std::vector<int> motor_haokong_id)
{

			for(int i_1 = 0;i_1<haokong_angle.size();i_1++){
				_Float32 dpi = pow(2,Motor_Vector[motor_haokong_id.at(i_1)-1].getEncodingRate())/ (2*M_PI);
				int result = dpi *haokong_angle.at(i_1);
				BYTE d1=(result&0xff);
				BYTE d2=(result>>8);
				BYTE d3=(result>>16);
				BYTE d4=(result>>24);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setID(Motor_Vector[motor_haokong_id.at(i_1)-1].getPdoID());
			Motor_Vector[motor_haokong_id.at(i_1)-1].setSendType(0);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setRemoteFlag(0);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setExternFlag(0);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setDataLen(4);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setData(0,d1);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setData(1,d2);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setData(2,d3);
			Motor_Vector[motor_haokong_id.at(i_1)-1].setData(3,d4);

			Motor_Vector[motor_haokong_id.at(i_1)-1].transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector[motor_haokong_id.at(i_1)-1].getFrame(), 1);		

				}
}





