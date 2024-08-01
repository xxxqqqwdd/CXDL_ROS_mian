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
    initslots();

    qnode.init();
    std::cout<<"ddddddd1"<<std::endl;
}

MainWindow::~MainWindow() {}


//my
void MainWindow::initslots()
{
    connect(ui.Btn_start ,SIGNAL(clicked()),&qnode,SLOT(ros_launch_start()));
//    connect(&qnode ,SIGNAL(motor_feedback_cb_signal(const cx_driver::feedback::ConstPtr& msg_p)),this,SLOT(motor_feedback_cb_(const cx_driver::feedback::ConstPtr& msg_p)));
connect(&qnode, SIGNAL(motor_feedback_cb(const cx_driver::feedbackConstPtr& msg_p)), this, SLOT(motor_feedback_cb_slot(const cx_driver::feedback::ConstPtr& msg_p)));

}




void MainWindow::on_ceshi_clicked()
{
    std::cout<<"ddddddd"<<std::endl;
}


void MainWindow::motor_feedback_cb_slot(const cx_driver::feedback::ConstPtr& msg_p)
{
std::cout<<"dddddddfw"<<std::endl;
}
