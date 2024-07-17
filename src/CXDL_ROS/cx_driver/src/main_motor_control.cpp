#include <iostream>
#include "ros/ros.h"
#include "cx_driver/controlcan.h"
#include "cx_driver/cxdriver.h"
#include "cx_driver/motor_control.h"
#include "cx_driver/motor_info.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "cx_driver/joint_angle.h"

#define BYTE unsigned char



void *receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
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
	//初始化SDO/PDO
    initCANOPENSdo(nh);
	//启动NMT
	startNMT();


	CAN_sync_interval = nh.param("CAN/sync_interval",10)/1000;
    ros::Timer State_Timer = nh.createTimer(ros::Duration(CAN_sync_interval), motor_timer_callback);

    pub_frame_info= nh.advertise<cx_driver::motor_info>("motor_frame_info",10);
    sub_frame_info = nh.subscribe<cx_driver::joint_angle>("joint_angle",10,joint_angle_cb);
	sub_motor_status = nh.subscribe<std_msgs::Bool>("motor_status",10,motor_status_cb);
    
    
    int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

    ros::spin();

    return 0;
}





