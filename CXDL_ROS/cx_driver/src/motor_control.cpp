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
	CAN_motor(msg_p->time,CAN_joint_Angle);


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
	CANOPEN_motor(msg_p->time,CANOPEN_joint_Angle);
}

void motor_status_cb(const std_msgs::Bool::ConstPtr& msg_p){
	if(msg_p->data == 1)
       motor_status_enable();
    else
       motor_status_disable();
}

void motor_status_enable()
{
	//ready_id
	std::vector<BYTE> ready_id{0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};
	//enable_id
	std::vector<BYTE> enable_id{0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};
	for(int v_m=0;v_m<Motor_Vector.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			//TODO：********************************
			//evo使能
		}
		else
		{
			if(v_m == 0||v_m == 1||v_m == 2||v_m == 6||v_m == 7||v_m == 8)
			{
				for(int j=0;j<=enable_id.size();j++)
				{
				Motor_Vector.at(v_m).setData(j,enable_id.at(j));
				}
				Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);
			}
			else
			{
				for(int j=0;j<=ready_id.size();j++)
				{
				Motor_Vector.at(v_m).setData(j,ready_id.at(j));
				}
				Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);
				
				for(int j=0;j<=enable_id.size();j++)
				{
				Motor_Vector.at(v_m).setData(j,enable_id.at(j));
				}
				Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);

			}
		}
	}	
}
void motor_status_disable()
{
	//disable_id
	std::vector<BYTE> disable_id{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
	for(int v_m=0;v_m<Motor_Vector.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			//TODO：********************************
			//evo失能
		}
		else
		{
			for(int j=0;j<=disable_id.size();j++)
			{
				Motor_Vector.at(v_m).setData(j,disable_id.at(j));
			}
			Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);
		}
	}		
}

	



void motor_timer_callback(const ros::TimerEvent){
	static int sy_j = 0;
if(is_sync_ok == TRUE)
{
	CAN_master.setSendType(0);
	CAN_master.setRemoteFlag(0);
	CAN_master.setExternFlag(0);
	CAN_master.setSdoID(0X80);
	CAN_master.setDataLen(0);
	CAN_master.transmit(CAN_frame_SDO_interval,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);


	VCI_CAN_OBJ pdo_send[all_pdo_storage.size()];
	for(int sy_i = 0;sy_i<all_pdo_storage.size();sy_i++)
	{
		if(sy_j<all_pdo_storage.at(sy_i).size())
		{
			pdo_send[sy_i].ID=all_pdo_storage.at(sy_i).at(sy_j).at(4);
			pdo_send[sy_i].DataLen=4;
			for(int i = 0; i < 4; i++)
			{
				pdo_send[sy_i].Data[i] = all_pdo_storage.at(sy_i).at(sy_j).at(i);
			}	
		}
	}
	CAN_master.transmit(CAN_frame_PDO_interval*1000,VCI_USBCAN2, 0, 0, pdo_send, all_pdo_storage.size());

	sy_j++;


	CAN_master.setSdoID(0X80);
	CAN_master.setDataLen(0);
	CAN_master.transmit(CAN_frame_SDO_interval,VCI_USBCAN2, 0, 0, &CAN_master.getFrame(), 1);
	
}

if(sy_j==all_pdo_storage.at(0).size())
{
	sy_j = 0;
	is_sync_ok = false;
	all_pdo_storage.clear();
}

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

Motor_Vector.push_back(motor_zhuoyu_1);
Motor_Vector.push_back(motor_zhuoyu_2);
Motor_Vector.push_back(motor_zhuoyu_3);
Motor_Vector.push_back(motor_haokong_1);
Motor_Vector.push_back(motor_evo_1);
Motor_Vector.push_back(motor_haokong_2);
Motor_Vector.push_back(motor_zhuoyu_4);
Motor_Vector.push_back(motor_zhuoyu_5);
Motor_Vector.push_back(motor_zhuoyu_6);
Motor_Vector.push_back(motor_haokong_3);
Motor_Vector.push_back(motor_evo_2);
Motor_Vector.push_back(motor_haokong_4);

//zhuoyu1_1
int zhuoyu1_sdoid ;
nh.param("MOTOR/zhuoyu1/SDO_ID",zhuoyu1_sdoid);
Motor_Vector.at(0).setSdoID(zhuoyu1_sdoid);
int zhuoyu1_pdoid ;
nh.param("MOTOR/zhuoyu1/PDO_ID",zhuoyu1_pdoid);
Motor_Vector.at(0).setPdoID(zhuoyu1_pdoid);
int zhuoyu1_encoding_rate ;
nh.param("MOTOR/zhuoyu1/Encoding_rat",zhuoyu1_encoding_rate);
Motor_Vector.at(0).setEncodingRate(zhuoyu1_encoding_rate);

//zhuoyu2_2
int zhuoyu2_sdoid ;
nh.param("MOTOR/zhuoyu2/SDO_ID",zhuoyu2_sdoid);
Motor_Vector.at(1).setSdoID(zhuoyu2_sdoid);
int zhuoyu2_pdoid ;
nh.param("MOTOR/zhuoyu2/PDO_ID",zhuoyu2_pdoid);
Motor_Vector.at(1).setPdoID(zhuoyu2_pdoid);
int zhuoyu2_encoding_rate ;
nh.param("MOTOR/zhuoyu2/Encoding_rat",zhuoyu2_encoding_rate);
Motor_Vector.at(1).setEncodingRate(zhuoyu2_encoding_rate);

//zhuoyu3_3
int zhuoyu3_sdoid ;
nh.param("MOTOR/zhuoyu3/SDO_ID",zhuoyu3_sdoid);
Motor_Vector.at(2).setSdoID(zhuoyu3_sdoid);
int zhuoyu3_pdoid ;
nh.param("MOTOR/zhuoyu3/PDO_ID",zhuoyu3_pdoid);
Motor_Vector.at(2).setPdoID(zhuoyu3_pdoid);
int zhuoyu3_encoding_rate ;
nh.param("MOTOR/zhuoyu3/Encoding_rat",zhuoyu3_encoding_rate);
Motor_Vector.at(2).setEncodingRate(zhuoyu3_encoding_rate);

//haokong1_4
int haokong1_sdoid ;
nh.param("MOTOR/haokong1/SDO_ID",haokong1_sdoid);
Motor_Vector.at(3).setSdoID(haokong1_sdoid);
int haokong1_pdoid ;
nh.param("MOTOR/haokong1/PDO_ID",haokong1_pdoid);
Motor_Vector.at(3).setPdoID(haokong1_pdoid);
int haokong1_encoding_rate ;
nh.param("MOTOR/haokong1/Encoding_rat",haokong1_encoding_rate);
Motor_Vector.at(3).setEncodingRate(haokong1_encoding_rate);

//evo1_5
int evo1_sdoid ;
nh.param("MOTOR/evo1/SDO_ID",evo1_sdoid);
Motor_Vector.at(4).setSdoID(evo1_sdoid);
int evo1_pdoid ;
nh.param("MOTOR/evo1/PDO_ID",evo1_pdoid);
Motor_Vector.at(4).setPdoID(evo1_pdoid);
int evo1_encoding_rate ;
nh.param("MOTOR/evo1/Encoding_rat",evo1_encoding_rate);
Motor_Vector.at(4).setEncodingRate(evo1_encoding_rate);

//haokong2_6
int haokong2_sdoid ;
nh.param("MOTOR/haokong2/SDO_ID",haokong2_sdoid);
Motor_Vector.at(5).setSdoID(haokong2_sdoid);
int haokong2_pdoid ;
nh.param("MOTOR/haokong2/PDO_ID",haokong2_pdoid);
Motor_Vector.at(5).setPdoID(haokong2_pdoid);
int haokong2_encoding_rate ;
nh.param("MOTOR/haokong2/Encoding_rat",haokong2_encoding_rate);
Motor_Vector.at(5).setEncodingRate(haokong2_encoding_rate);


//zhuoyu4_7
int zhuoyu4_sdoid ;
nh.param("MOTOR/zhuoyu4/SDO_ID",zhuoyu4_sdoid);
Motor_Vector.at(6).setSdoID(zhuoyu4_sdoid);
int zhuoyu4_pdoid ;
nh.param("MOTOR/zhuoyu4/PDO_ID",zhuoyu4_pdoid);
Motor_Vector.at(6).setPdoID(zhuoyu4_pdoid);
int zhuoyu4_encoding_rate ;
nh.param("MOTOR/zhuoyu4/Encoding_rat",zhuoyu4_encoding_rate);
Motor_Vector.at(6).setEncodingRate(zhuoyu4_encoding_rate);

//zhuoyu5_8
int zhuoyu5_sdoid ;
nh.param("MOTOR/zhuoyu5/SDO_ID",zhuoyu5_sdoid);
Motor_Vector.at(7).setSdoID(zhuoyu5_sdoid);
int zhuoyu5_pdoid ;
nh.param("MOTOR/zhuoyu5/PDO_ID",zhuoyu5_pdoid);
Motor_Vector.at(7).setPdoID(zhuoyu5_pdoid);
int zhuoyu5_encoding_rate ;
nh.param("MOTOR/zhuoyu5/Encoding_rat",zhuoyu5_encoding_rate);
Motor_Vector.at(7).setEncodingRate(zhuoyu5_encoding_rate);

//zhuoyu6_9
int zhuoyu6_sdoid ;
nh.param("MOTOR/zhuoyu6/SDO_ID",zhuoyu6_sdoid);
Motor_Vector.at(8).setSdoID(zhuoyu6_sdoid);
int zhuoyu6_pdoid ;
nh.param("MOTOR/zhuoyu6/PDO_ID",zhuoyu6_pdoid);
Motor_Vector.at(8).setPdoID(zhuoyu6_pdoid);
int zhuoyu6_encoding_rate ;
nh.param("MOTOR/zhuoyu6/Encoding_rat",zhuoyu6_encoding_rate);
Motor_Vector.at(8).setEncodingRate(zhuoyu6_encoding_rate);

//haokong3_10
int haokong3_sdoid ;
nh.param("MOTOR/haokong3/SDO_ID",haokong3_sdoid);
Motor_Vector.at(9).setSdoID(haokong3_sdoid);
int haokong3_pdoid ;
nh.param("MOTOR/haokong3/PDO_ID",haokong3_pdoid);
Motor_Vector.at(9).setPdoID(haokong3_pdoid);
int haokong3_encoding_rate ;
nh.param("MOTOR/haokong3/Encoding_rat",haokong3_encoding_rate);
Motor_Vector.at(9).setEncodingRate(haokong3_encoding_rate);

//evo2_11
int evo2_sdoid ;
nh.param("MOTOR/evo2/SDO_ID",evo2_sdoid);
Motor_Vector.at(10).setSdoID(evo2_sdoid);
int evo2_pdoid ;
nh.param("MOTOR/evo2/PDO_ID",evo2_pdoid);
Motor_Vector.at(10).setPdoID(evo2_pdoid);
int evo2_encoding_rate ;
nh.param("MOTOR/evo2/Encoding_rat",evo2_encoding_rate);
Motor_Vector.at(10).setEncodingRate(evo2_encoding_rate);

//haokong4_12
int haokong4_sdoid ;
nh.param("MOTOR/haokong4/SDO_ID",haokong4_sdoid);
Motor_Vector.at(11).setSdoID(haokong4_sdoid);
int haokong4_pdoid ;
nh.param("MOTOR/haokong4/PDO_ID",haokong4_pdoid);
Motor_Vector.at(11).setPdoID(haokong4_pdoid);
int haokong4_encoding_rate ;
nh.param("MOTOR/haokong4/Encoding_rat",haokong4_encoding_rate);
Motor_Vector.at(11).setEncodingRate(haokong4_encoding_rate);


//CAN
nh.param("CAN/frame_SDO_interval",CAN_frame_SDO_interval);
nh.param("CAN/frame_PDO_interval",CAN_frame_PDO_interval);
nh.param("CAN/sync_interval",CAN_sync_interval);


is_zhuoyu_pdo_ok=false;
is_haokong_pdo_ok=false;
is_sync_ok =false;
}


void initCANOPENSdo(ros::NodeHandle nh){


	//TODO：缺少evo的SDO********************************
	for(int v_m=0;v_m<Motor_Vector.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			
		}
	}

	for(int v_m=0;v_m<Motor_Vector.size();v_m++){
		if(v_m == 4||v_m == 10)
		{
			continue;
		}
		Motor_Vector.at(v_m).setSendType(0);
		Motor_Vector.at(v_m).setRemoteFlag(0);
		Motor_Vector.at(v_m).setExternFlag(0);
		Motor_Vector.at(v_m).setDataLen(8);
	}
	//TODO：zhuoyu和haokong不一样   230   同步********************************
	//循环位置模式帧
	std::vector<BYTE> mode_set{0x2F,0x60,0x60,0x00,0x08,0x00,0x00,0x00};
	zhuoyu_sdo.push_back(mode_set);
	haokong_sdo.push_back(mode_set);
	//电机运行间隔
	std::vector<BYTE> moteo_interval{0x2F,0xC2,0x60,0x01,0x1E,0x00,0x00,0x00};
	zhuoyu_sdo.push_back(moteo_interval);
	haokong_sdo.push_back(moteo_interval);
	//tpdo1_1
	std::vector<BYTE> tpdo1_1{0x23,0x00,0x18,0x01,0x03,0x18,0x00,0x80};
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
	std::vector<BYTE> tpdo1_7{0x23,0x00,0x18,0x01,0x03,0x18,0x00,0x00};
	zhuoyu_sdo.push_back(tpdo1_7);
	haokong_sdo.push_back(tpdo1_7);
	//tpdo2_1
	std::vector<BYTE> tpdo2_1{0x23,0x01,0x18,0x01,0x03,0x28,0x00,0x80};
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
	std::vector<BYTE> tpdo2_8{0x23,0x01,0x18,0x01,0x03,0x28,0x00,0x00};
	zhuoyu_sdo.push_back(tpdo2_8);
	haokong_sdo.push_back(tpdo2_8);
	//rpdo1_1
	std::vector<BYTE> rpdo1_1{0x23,0x00,0x14,0x01,0x03,0x02,0x00,0x80};
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
	std::vector<BYTE> rpdo1_4{0x23,0x00,0x16,0x01,0x20,0x00,0x7A,0x60};
	zhuoyu_sdo.push_back(rpdo1_4);
	haokong_sdo.push_back(rpdo1_4);
	//rpdo1_5
	std::vector<BYTE> rpdo1_5{0x2F,0x00,0x16,0x00,0x01,0x00,0x00,0x00};
	zhuoyu_sdo.push_back(rpdo1_5);
	haokong_sdo.push_back(rpdo1_5);
	//rpdo1_6
	std::vector<BYTE> rpdo1_6{0x23,0x00,0x14,0x01,0x03,0x02,0x00,0x00};
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
			for(int sdo_row_i=0;sdo_row_i<=zhuoyu_sdo.size();sdo_row_i++){
				for(int j=0;j<=zhuoyu_sdo.at(sdo_row_i).size();j++)
				{
					Motor_Vector.at(v_m).setData(j,zhuoyu_sdo.at(sdo_row_i).at(j));
				}
				Motor_Vector.at(v_m).setID(Motor_Vector.at(v_m).getSdoID());
				Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);
			}
		}

		if(v_m == 3||v_m == 5||v_m == 9||v_m == 11)
		{
			for(int sdo_row_i=0;sdo_row_i<=haokong_sdo.size();sdo_row_i++){
				for(int j=0;j<=haokong_sdo.at(sdo_row_i).size();j++)
				{
					Motor_Vector.at(v_m).setData(j,haokong_sdo.at(sdo_row_i).at(j));
				}
				Motor_Vector.at(v_m).setID(Motor_Vector.at(v_m).getSdoID());
				Motor_Vector.at(v_m).transmit(CAN_frame_SDO_interval*1000,VCI_USBCAN2, 0, 0, &Motor_Vector.at(v_m).getFrame(), 1);
			}
		}
	}
}



void startNMT(){
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




void CAN_motor(double time,std::vector<double> CAN_joint_Angle)
{
	std::vector<int> motor_evo_id{5,11};
	evo_motor_control(time ,CAN_joint_Angle,motor_evo_id);
}

void CANOPEN_motor(double time,std::vector<double> CANOPEN_motor_joint_Angle)
{
	std::vector<double> zhuoyu_motor{CANOPEN_motor_joint_Angle.at(0),
									CANOPEN_motor_joint_Angle.at(1),
									CANOPEN_motor_joint_Angle.at(2),
									CANOPEN_motor_joint_Angle.at(5),
									CANOPEN_motor_joint_Angle.at(6),
									CANOPEN_motor_joint_Angle.at(7)};
	std::vector<int> motor_zhuoyu_id{1,2,3,7,8,9};
	zhuoyu_motor_control(time ,zhuoyu_motor,motor_zhuoyu_id);
	std::vector<double> haokong_motor{CANOPEN_motor_joint_Angle.at(3),
									CANOPEN_motor_joint_Angle.at(4),
									CANOPEN_motor_joint_Angle.at(8),
									CANOPEN_motor_joint_Angle.at(9),};
	std::vector<int> motor_haokong_id{4,6,10,12};
	haokong_motor_control(time ,haokong_motor,motor_haokong_id);
}


std::vector<int> section(double time ,int value)
{
	std::vector<int> section_Data;
	int frequency = time/CAN_sync_interval;
	int remainder = value % frequency;

	if (remainder != 0) 
	{
		int partSize = value / (frequency-1);		
		
		int currentSum = 0;
		for (int i = 0; i < frequency; ++i) { 
		currentSum += partSize;
		section_Data.push_back(currentSum);
		}
		currentSum += remainder;
		section_Data.push_back(currentSum);
	}
	else
	{
		int partSize = value / (frequency);		
		int currentSum = 0;
		for (int i = 0; i < frequency; ++i) { 
		currentSum += partSize;
		section_Data.push_back(currentSum);
		}
	}

	return section_Data;
}


void evo_motor_control(double time,std::vector<double> evo_angle,std::vector<int> motor_evo_id)
{	
	
}
void zhuoyu_motor_control(double time,std::vector<double> zhuoyu_angle,std::vector<int> motor_zhuoyu_id)
{
	is_zhuoyu_pdo_ok=false;
	for(int i_1=0;i_1<motor_zhuoyu_id.size();i_1++)
	{
		Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).setID(Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).getPdoID());
		Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).setSendType(0);
		Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).setRemoteFlag(0);
		Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).setExternFlag(0);
		Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).setDataLen(4);

	_Float32 dpi = pow(2, Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).getEncodingRate())/ (2*M_PI);
	dpi = dpi *zhuoyu_angle.at(i_1);
	// std::cout<<dpi<<std::endl;
	int result = dpi;
	std::vector<int> nums = section(time,result);

	std::vector<std::vector<BYTE>> pdo_storage;
	for (int i_2=0;i_2<nums.size();i_2++) {

	BYTE d1=(nums.at(i_2)&0xff);
	BYTE d2=(nums.at(i_2)>>8);
	BYTE d3=(nums.at(i_2)>>16);
	BYTE d4=(nums.at(i_2)>>24);

	std::vector<BYTE> single_pdo_storage{d1,d2,d3,d4,Motor_Vector.at(motor_zhuoyu_id.at(i_1)-1).getPdoID()};
	pdo_storage.push_back(single_pdo_storage);
	}
	all_pdo_storage.push_back(pdo_storage);

	}
	is_zhuoyu_pdo_ok = true;
	if(is_zhuoyu_pdo_ok == true ||is_haokong_pdo_ok == true )
	{
		is_sync_ok =true;
	}
}
void haokong_motor_control(double time,std::vector<double> haokong_angle,std::vector<int> motor_haokong_id)
{
	is_haokong_pdo_ok = false;
	for(int i_1=0;i_1<motor_haokong_id.size();i_1++)
	{
		Motor_Vector.at(motor_haokong_id.at(i_1)-1).setID(Motor_Vector.at(motor_haokong_id.at(i_1)-1).getPdoID());
		Motor_Vector.at(motor_haokong_id.at(i_1)-1).setSendType(0);
		Motor_Vector.at(motor_haokong_id.at(i_1)-1).setRemoteFlag(0);
		Motor_Vector.at(motor_haokong_id.at(i_1)-1).setExternFlag(0);
		Motor_Vector.at(motor_haokong_id.at(i_1)-1).setDataLen(4);

	_Float32 dpi = pow(2, Motor_Vector.at(motor_haokong_id.at(i_1)-1).getEncodingRate())/ (2*M_PI);
	dpi = dpi *haokong_angle.at(i_1);
	// std::cout<<dpi<<std::endl;
	int result = dpi;
	std::vector<int> nums = section(time,result);

	std::vector<std::vector<BYTE>> pdo_storage;
	for (int i_2=0;i_2<nums.size();i_2++) {

	BYTE d1=(nums.at(i_2)&0xff);
	BYTE d2=(nums.at(i_2)>>8);
	BYTE d3=(nums.at(i_2)>>16);
	BYTE d4=(nums.at(i_2)>>24);

	std::vector<BYTE> single_pdo_storage{d1,d2,d3,d4};
	pdo_storage.push_back(single_pdo_storage);
	}
	all_pdo_storage.push_back(pdo_storage);
}
is_haokong_pdo_ok = true;
	if(is_zhuoyu_pdo_ok == true ||is_haokong_pdo_ok == true )
	{
		is_sync_ok =true;
	}

}

