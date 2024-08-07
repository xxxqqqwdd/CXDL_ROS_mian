#ifndef MYMOTORTHREAD_H
#define MYMOTORTHREAD_H

#include <QObject>
#include <QRunnable>
#include <QDebug>
#include <ros/ros.h>
#include "../../../devel/include/cx_driver/joint_angle.h"



class MyMotorThread :  public QObject
{
    Q_OBJECT

public:
    explicit MyMotorThread(QObject *parent = nullptr);



signals:
    void moter_ceshi_launch_signal(cx_driver::joint_angle angles);

public slots:
    void motor_working(uint a);


private:

    uint m_motorid;

    //
    cx_driver::joint_angle angles1;

};

#endif // MYMOTORTHREAD_H



