/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cx_qt/main_window.hpp"
#include <QString>

#include <QDebug>
#include <ros/ros.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/



using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    initUI();
    initThread();
    initslots();

    qnode.init();

}

MainWindow::~MainWindow() {}

void MainWindow::initUI()
{
    //时间戳
    stamp_name = new QLabel;
    stamp = new QLabel;
    stamp_name->setFixedSize(60, 20);
    stamp_name->setText("时间戳：");
    ui.statusbar->addWidget((stamp_name));
    ui.statusbar->addWidget((stamp));

    //单选按钮组
    m_btnGroup1 = new QButtonGroup;

    m_btnGroup1->addButton(ui.rBtn_v1,1);
    m_btnGroup1->addButton(ui.rBtn_v2,2);
    m_btnGroup1->addButton(ui.rBtn_v3,3);
    m_btnGroup1->addButton(ui.rBtn_v4,4);
    m_btnGroup1->addButton(ui.rBtn_v5,5);
    m_btnGroup1->addButton(ui.rBtn_v6,6);
    m_btnGroup1->addButton(ui.rBtn_v7,7);
    m_btnGroup1->addButton(ui.rBtn_v8,8);
    m_btnGroup1->addButton(ui.rBtn_v9,9);
    m_btnGroup1->addButton(ui.rBtn_v10,10);
    m_btnGroup1->addButton(ui.rBtn_v11,11);
    m_btnGroup1->addButton(ui.rBtn_v12,12);
    m_btnGroup1->addButton(ui.rBtn_acc1,1);
    m_btnGroup1->addButton(ui.rBtn_acc2,2);
    m_btnGroup1->addButton(ui.rBtn_acc3,3);
    m_btnGroup1->addButton(ui.rBtn_acc4,4);
    m_btnGroup1->addButton(ui.rBtn_acc6,6);
    m_btnGroup1->addButton(ui.rBtn_acc7,7);
    m_btnGroup1->addButton(ui.rBtn_acc8,8);
    m_btnGroup1->addButton(ui.rBtn_acc9,9);
    m_btnGroup1->addButton(ui.rBtn_acc10,10);
    m_btnGroup1->addButton(ui.rBtn_acc12,12);
    m_btnGroup1->addButton(ui.rBtn_dec1,1);
    m_btnGroup1->addButton(ui.rBtn_dec2,2);
    m_btnGroup1->addButton(ui.rBtn_dec3,3);
    m_btnGroup1->addButton(ui.rBtn_dec4,4);
    m_btnGroup1->addButton(ui.rBtn_dec6,6);
    m_btnGroup1->addButton(ui.rBtn_dec7,7);
    m_btnGroup1->addButton(ui.rBtn_dec8,8);
    m_btnGroup1->addButton(ui.rBtn_dec9,9);
    m_btnGroup1->addButton(ui.rBtn_dec10,10);
    m_btnGroup1->addButton(ui.rBtn_dec12,12);

}


void MainWindow::initslots()
{

    connect(ui.btn_start ,SIGNAL(clicked()),&qnode,SLOT(ros_launch_start()));
//    connect(ui.btn_start_1 ,SIGNAL(clicked()),&qnode,SLOT(ros_launch_start()));

    //电机反馈
    connect(&qnode, &QNode::motorFeedbackReceived, this, &MainWindow::motor_feedback_cb_slot);

    //电机使能
    connect(this,&MainWindow::on_btn_motorenable_clicked_signal,&qnode,&QNode::on_btn_motorenable_clicked_slot);

    //电机失能
    connect(this,&MainWindow::on_btn_motordisable_clicked_signal,&qnode,&QNode::on_btn_motordisable_clicked_slot);

    //启动线程
    connect(this,&MainWindow::thread_start_t4,motor_thread,&MyMotorThread::motor_working);

    //线程处理完角度给qnode
    connect(motor_thread,&MyMotorThread::moter_ceshi_launch_signal,&qnode,&QNode::moter_ceshi_launch_slot);

    //01模式控制升降
    connect(this,&MainWindow::on_btn_set_H_clicked_signal,&qnode,&QNode::on_btn_set_H_clicked_slot);

    //按钮组
    connect(m_btnGroup1,SIGNAL(bool checked),this,SLOT(set_param()));

}


//初始化线程
void MainWindow::initThread()
{
    motor_thread = new MyMotorThread;
    body1_thread = new MyMotorThread;
    body2_thread = new MyMotorThread;
    body3_thread = new MyMotorThread;
     t1 = new QThread;
     t2 = new QThread;
     t3 = new QThread;
     t4 = new QThread;


    body1_thread->moveToThread(t1);
    body2_thread->moveToThread(t2);
    body3_thread->moveToThread(t3);
    motor_thread->moveToThread(t4);
}


void MainWindow::motor_feedback_cb_slot(cx_driver::feedback* msg)
{
    //左臂
    ui.lineEdit_joint101_1->setAlignment(Qt::AlignRight);
    ui.lineEdit_joint101_1->setText(QString::number(msg->position[0],'f',6));
    ui.lineEdit_joint101_2->setText(QString::number(msg->velocity[0],'f',6));
    ui.lineEdit_joint101_3->setText(QString::number(msg->torque[0],'f',6));
    ui.lineEdit_joint102_1->setText(QString::number(msg->position[1],'f',6));
    ui.lineEdit_joint102_2->setText(QString::number(msg->velocity[1],'f',6));
    ui.lineEdit_joint102_3->setText(QString::number(msg->torque[1],'f',6));
    ui.lineEdit_joint103_1->setText(QString::number(msg->position[2],'f',6));
    ui.lineEdit_joint103_2->setText(QString::number(msg->velocity[2],'f',6));
    ui.lineEdit_joint103_3->setText(QString::number(msg->torque[2],'f',6));
    ui.lineEdit_joint104_1->setText(QString::number(msg->position[3],'f',6));
    ui.lineEdit_joint104_2->setText(QString::number(msg->velocity[3],'f',6));
    ui.lineEdit_joint104_3->setText(QString::number(msg->torque[3],'f',6));
    ui.lineEdit_joint105_1->setText(QString::number(msg->position[4],'f',6));
    ui.lineEdit_joint105_2->setText(QString::number(msg->velocity[4],'f',6));
    ui.lineEdit_joint105_3->setText(QString::number(msg->torque[4],'f',6));
    ui.lineEdit_joint106_1->setText(QString::number(msg->position[5],'f',6));
    ui.lineEdit_joint106_2->setText(QString::number(msg->velocity[5],'f',6));
    ui.lineEdit_joint106_3->setText(QString::number(msg->torque[5],'f',6));
    //右臂
    ui.lineEdit_joint201_1->setText(QString::number(msg->position[6],'f',6));
    ui.lineEdit_joint201_2->setText(QString::number(msg->velocity[6],'f',6));
    ui.lineEdit_joint201_3->setText(QString::number(msg->torque[6],'f',6));
    ui.lineEdit_joint202_1->setText(QString::number(msg->position[7],'f',6));
    ui.lineEdit_joint202_2->setText(QString::number(msg->velocity[7],'f',6));
    ui.lineEdit_joint202_3->setText(QString::number(msg->torque[7],'f',6));
    ui.lineEdit_joint203_1->setText(QString::number(msg->position[8],'f',6));
    ui.lineEdit_joint203_2->setText(QString::number(msg->velocity[8],'f',6));
    ui.lineEdit_joint203_3->setText(QString::number(msg->torque[8],'f',6));
    ui.lineEdit_joint204_1->setText(QString::number(msg->position[9],'f',6));
    ui.lineEdit_joint204_2->setText(QString::number(msg->velocity[9],'f',6));
    ui.lineEdit_joint204_3->setText(QString::number(msg->torque[9],'f',6));
    ui.lineEdit_joint205_1->setText(QString::number(msg->position[10],'f',6));
    ui.lineEdit_joint205_2->setText(QString::number(msg->velocity[10],'f',6));
    ui.lineEdit_joint205_3->setText(QString::number(msg->torque[10],'f',6));
    ui.lineEdit_joint206_1->setText(QString::number(msg->position[11],'f',6));
    ui.lineEdit_joint206_2->setText(QString::number(msg->velocity[11],'f',6));
    ui.lineEdit_joint206_3->setText(QString::number(msg->torque[11],'f',6));


    //升降架
    double L1= msg->position[12];
    double L2= msg->position[13];
    double H1=sin(0.5*M_PI-acos((255625-pow((505.5937-L1),2))/75000))*500;
    double H2=sin(0.5*M_PI-acos((255625-pow((505.5937-L2),2))/75000))*500;
    ui.lineEdit_shengjiang_L1->setText(QString::number(L1,'f',6));
    ui.lineEdit_shengjiang_L2->setText(QString::number(L2,'f',6));
    ui.lineEdit_shengjiang_H->setText(QString::number(H1+H2+155+85,'f',6));

    //里程计
    ui.lineEdit_mileage_left->setText(QString::number(msg->mileage[0],'f',6));
    ui.lineEdit_mileage_right->setText(QString::number(msg->mileage[1],'f',6));



    stamp_name->setText(QString::asprintf("时间戳："));
    stamp->setText(QString::asprintf("%f",msg->stamp.toSec()));

}




void MainWindow::on_btn_ceshi_clicked()
{


    uint a = ui.comboBox_motorid->currentText().toUInt();
    emit thread_start_t4(a);
    t4->start();
}

//电机使能
void MainWindow::on_btn_motorenable_clicked()
{
    emit on_btn_motorenable_clicked_signal();
}

//电机失能
void MainWindow::on_btn_motordisable_clicked()
{
    emit on_btn_motordisable_clicked_signal();
}

//控制升降
void MainWindow::on_btn_set_H_clicked()
{
    double s = ui.lineEdit_shengjiang_set_H->text().toDouble();

    emit on_btn_set_H_clicked_signal(s);
}

void MainWindow::set_param()
{
    qDebug()<<"wwwwwwwwwwwwww";
}

