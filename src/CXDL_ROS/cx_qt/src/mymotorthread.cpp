#include "../include/cx_qt/mymotorthread.h"

MyMotorThread::MyMotorThread(QObject *parent) : QObject(parent)
{

}


void MyMotorThread::motor_working(uint a)
{
    ros::Rate loop_rate(33);

    for (int i=0;i<942;i++){
        double angle_1=sin(i/300.0)*2*M_PI;
        angles1.left_arm_joint[a-1]= angle_1;
        emit moter_ceshi_launch_signal(angles1);
        loop_rate.sleep();

    }
}









