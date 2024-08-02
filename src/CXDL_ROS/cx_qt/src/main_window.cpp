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
#include    <QString>
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
    initslots();

    qnode.init();

}

MainWindow::~MainWindow() {}

void MainWindow::initUI()
{
    stamp_name = new QLabel;
    stamp = new QLabel;
    stamp_name->setFixedSize(60, 20);
    stamp_name->setText("时间戳：");
    ui.statusbar->addWidget((stamp_name));
    ui.statusbar->addWidget((stamp));


}


void MainWindow::initslots()
{

    connect(ui.btn_start ,SIGNAL(clicked()),&qnode,SLOT(ros_launch_start()));

    //电机反馈
    connect(&qnode, &QNode::motorFeedbackReceived, this, &MainWindow::motor_feedback_cb_slot);

    //电机测试
    connect(this,&MainWindow::moter_ceshi,&qnode,&QNode::moter_ceshi_);


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


    //右壁
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



    stamp_name->setText(QString::asprintf("时间戳："));


    stamp->setText(QString::asprintf("%f",msg->stamp.toSec()));

}




void MainWindow::on_btn_ceshi_clicked()
{
    unsigned long a = ui.lineEdit_1->text().toULong();

    emit moter_ceshi(a);
}
