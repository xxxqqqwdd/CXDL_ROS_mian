
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

     void initUI();
    void initslots();

private slots:
    void motor_feedback_cb_slot(cx_driver::feedback*);

    void on_btn_ceshi_clicked();

Q_SIGNALS:

signals:
    void moter_ceshi(int a);


private:
	Ui::MainWindowDesign ui;
	QNode qnode; 

    QLabel *stamp;
    QLabel *stamp_name;


};



#endif // CX_QT_MAIN_WINDOW_H
