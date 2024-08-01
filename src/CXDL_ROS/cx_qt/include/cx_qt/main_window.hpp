
#ifndef CX_QT_MAIN_WINDOW_H
#define CX_QT_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

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





    //my
    void initslots();

private slots:


    void on_ceshi_clicked();
    void motor_feedback_cb_slot(const cx_driver::feedback::ConstPtr& msg_p);

Q_SIGNALS:






private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};



#endif // CX_QT_MAIN_WINDOW_H
