
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

namespace CX_QT {

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



private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace CX_QT

#endif // CX_QT_MAIN_WINDOW_H
