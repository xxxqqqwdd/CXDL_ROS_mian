/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/cx_qt/main_window.hpp"
//#include "../../../devel/include/cx_driver/joint_angle_08.h"


/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/

    QApplication app(argc, argv);
    QTextCodec* code = QTextCodec::codecForName("GBK");
    QTextCodec::setCodecForLocale(code);

    qRegisterMetaType<cx_driver::joint_angle_08>("cx_driver::joint_angle_08");




    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
