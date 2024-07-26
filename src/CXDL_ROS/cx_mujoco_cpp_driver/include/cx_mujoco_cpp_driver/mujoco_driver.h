#ifndef _MUJOCO_DRIVER_H_
#define _MUJOCO_DRIVER_H_


#include "mujoco/mujoco.h"
void set_torque_control(const mjModel* m,int actuator_no,int flag);



void set_position_servo(const mjModel* m,int actuator_no,double kp);


void set_velocity_servo(const mjModel* m,int actuator_no,double kv);


void init_controller(const mjModel* m, mjData* d);


void mycontroller(const mjModel* m, mjData* d);


#endif
