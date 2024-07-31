#include "cx_mujoco_cpp_driver/mujoco_driver.h"
#include <iostream>


/******************************/
void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0]=0;
  else
    m->actuator_gainprm[10*actuator_no+0]=1;
}
/******************************/


/******************************/
void set_position_servo(const mjModel* m,int actuator_no,double kp)
{
  m->actuator_gainprm[10*actuator_no+0]=kp;
  m->actuator_biasprm[10*actuator_no+1]=-kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
{
  m->actuator_gainprm[10*actuator_no+0]=kv;
  m->actuator_biasprm[10*actuator_no+2]=-kv;
}
/******************************/

//**************************
void init_controller(const mjModel* m, mjData* d)
{


   d->qpos[0]=0;
   int actuator_no=1;
   int flag=0;
   double kp=100;
   set_position_servo( m, actuator_no, kp);
   actuator_no=2;
   double kv=0.1;
    set_velocity_servo( m,actuator_no, kv);

//  set_torque_control( m,actuator_no, flag);
}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{

  d->ctrl[1]=2;
//  std::cout<<d->sensordata[0]<<std::endl;




}




















