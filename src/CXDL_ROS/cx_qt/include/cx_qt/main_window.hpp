
#ifndef CX_QT_MAIN_WINDOW_H
#define CX_QT_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "mymotorthread.h"
#include <QThread>

/*****************************************************************************
** Namespace
*****************************************************************************/


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    //初始化UI
     void initUI();
     //初始化信号和槽
    void initslots();
    //初始化线程
     void initThread();

private slots:
    void motor_feedback_cb_slot(cx_driver::feedback*);

    void on_btn_ceshi_clicked();

    //电机使能
    void on_btn_motorenable_clicked();


    //电机失能能
    void on_btn_motordisable_clicked();




Q_SIGNALS:

signals:
    //电机使能
    void on_btn_motorenable_clicked_signal();

    //电机失能
    void on_btn_motordisable_clicked_signal();

    //启动线程
    void thread_start(uint a);


private:
	Ui::MainWindowDesign ui;
	QNode qnode; 

    QLabel *stamp;
    QLabel *stamp_name;

    //电机线程
     MyMotorThread* motor_thread;
     QThread* t1;



};



#endif // CX_QT_MAIN_WINDOW_H
