// File:          Warlocks_LineFollowing.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <string>
#define TIME_STEP 16

using namespace webots;
using namespace std;

#define MAX_SPEED 10
#define MEDIUM_SPEED 2
#define SENSOR_THRESHOLD 115

float IR_Reading[8]; 
int IR_Status[8];
int IR_Weight[8]= {-8000,-4000,-2000,-1000,1000,2000,4000,8000};

//For PID
float p_err = 0.0;
float Kp =5;
float Kd =0.1;
float Ki =0;
float Integral =0.0;


float err = 0;
float P;
float D;
float Correction;

float left_speed , right_speed;
float I;

  //Creating Robot Instance
Robot *robot = new Robot();
Motor *left_motor = robot->getMotor("left_motor");
Motor *right_motor = robot->getMotor("right_motor");

//Arm Motors
Motor *v_linear = robot->getMotor("v_linear_motor");
Motor *hr_linear = robot->getMotor("hr_linear_motor");
Motor *hl_linear = robot->getMotor("hl_linear_motor");

DistanceSensor *ir1 = robot->getDistanceSensor("ir1");
DistanceSensor *ir2 = robot->getDistanceSensor("ir2");
DistanceSensor *ir3 = robot->getDistanceSensor("ir3");
DistanceSensor *ir4 = robot->getDistanceSensor("ir4");
DistanceSensor *ir5 = robot->getDistanceSensor("ir5");
DistanceSensor *ir6 = robot->getDistanceSensor("ir6");
DistanceSensor *ir7 = robot->getDistanceSensor("ir7");
DistanceSensor *ir8 = robot->getDistanceSensor("ir8");
  

void delay(int Time)
   {
  float current_time_1 = float(robot->getTime());
  float current_time_2= float(robot->getTime());
  do {
    current_time_2 = float(robot->getTime());
    robot->step(1);
  } while(current_time_2 < (current_time_1 + Time));
   }
/////////////////////////////////////////////////////////////////////////////////
void Init_Motors()
  {
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  
  //Set Initial Motor Speed to 0
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
  
  }
///////////////////////////////////////////////////////////////////////////////
void Init_IR_Sensors()
  { 
  ir1->enable(TIME_STEP);
  ir2->enable(TIME_STEP);
  ir3->enable(TIME_STEP);
  ir4->enable(TIME_STEP);
  ir5->enable(TIME_STEP);
  ir6->enable(TIME_STEP);
  ir7->enable(TIME_STEP);
  ir8->enable(TIME_STEP);
  }
////////////////////////////////////////////////////////////////////////////////  
float PID(float pr_err)
 {
  err = 0;
  for(int i=0;i<8;i++)
  {
    err +=IR_Weight[i]*IR_Status[i];
  }
  
  P = Kp*err;
  D = Kd*(err-pr_err);
  I = Integral+Ki*err;
  Correction = (P+D+I)/1000;
  //cout <<Correction<<endl;
  
  left_speed = MEDIUM_SPEED+Correction;
  right_speed = MEDIUM_SPEED-Correction;
  
  if(left_speed<0.0){left_speed=0.0;}
  if(left_speed>MAX_SPEED){left_speed=MAX_SPEED;}
  if(right_speed<0.0){right_speed=0.0;}
  if(right_speed>MAX_SPEED){right_speed=MAX_SPEED;}
  
  return err;
 } 
//////////////////////////////////////////////////////////////////////////
void Read_IR_Sensors()
  {
   //Read Values from Sensors
   IR_Reading[0] = ir1->getValue();
   IR_Reading[1] = ir2->getValue();
   IR_Reading[2] = ir3->getValue();
   IR_Reading[3] = ir4->getValue();
   IR_Reading[4] = ir5->getValue();
   IR_Reading[5] = ir6->getValue();
   IR_Reading[6] = ir7->getValue();
   IR_Reading[7] = ir8->getValue();
  
  //Print IR Readings
  cout <<"{"<<endl;
  for(int i=0;i<8;i++)
  {
   cout << IR_Reading[i] <<" ";
  } 
  cout <<"}\n"<<endl;  
    
  //Check the Sensors detect the path or not
 for(int i=0;i<8;i++)
  {
    if(IR_Reading[i]<SENSOR_THRESHOLD)
    {
      IR_Status[i] =0;
    }
    else
    {
      IR_Status[i] =1;
    }
  }
  }
//////////////////////////////////////////////////////////////////////////////
void Arm_horizontal(float d)
{
  hl_linear->setPosition(d);
  hr_linear->setPosition(-1*d);
}  

void Arm_verticle(float d)
{
  v_linear->setPosition(d);
} 

///////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

  Init_IR_Sensors();
  Init_Motors();
  Arm_horizontal(0.04);
  Arm_verticle(0.1);
while (robot->step(TIME_STEP) != -1)
  {
  Read_IR_Sensors(); 
  p_err = PID(p_err);
  
 // cout << left_speed <<" "<< right_speed <<endl;
  
  left_motor->setVelocity(left_speed);
  right_motor->setVelocity(right_speed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
