#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 16

#define normal_speed 6

using namespace webots;

Robot *robot = new Robot();
Motor *motors[2] = {robot->getMotor("left_motor"),robot->getMotor("right_motor")};
DistanceSensor *us[3];
DistanceSensor *ir[8];
Camera *cm = robot->getCamera("cam");



char irNames[8][5] = {"ir1","ir2","ir3","ir4","ir5","ir6","ir7","ir8"};
char usNames[3][10] = {"us_left","us_front","us_right"};
double left_motor =0.0;
double right_motor=0.0;



void delay(float t);
void turn_left();
void turn_right();
void turn_back();
void follow_continous_line();
void solve_maze();

int main(int argc, char **argv) {
 
  for (int i = 0; i < 2; i++) {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }
  
  cm->enable(TIME_STEP);
  while (robot->step(TIME_STEP) != -1) {
    follow_continous_line();
    delay(500);
    solve_maze(); 
       
  }
  
}
  














void delay(float t){
  float current = robot->getTime();
  float final  = current+(t/1000);
  while (final>robot->getTime()){
    robot->step(1);
  }
}

void turn_left(){
   motors[0]->setVelocity(0);
   motors[1]->setVelocity(8);
   delay(1120);
   motors[0]->setVelocity(0.0);
   motors[1]->setVelocity(0.0);
}

void turn_right(){
   motors[0]->setVelocity(10);
   motors[1]->setVelocity(-6);
   delay(560);
   motors[0]->setVelocity(0.0);
   motors[1]->setVelocity(0.0);
}

void turn_back(){
   motors[0]->setVelocity(-8);
   motors[1]->setVelocity(8);
   delay(1120);
   motors[0]->setVelocity(0.0);
   motors[1]->setVelocity(0.0);
}




void follow_continous_line(){

  int ir_readings[8];
  double kp=1;
  double kd=0.2;
  double ki=0.02;
  double prevoious_error = 0.0;
  double integral = 0.0;
  int coefficient[8]={-4000,-3000,-2000,-1000,1000,2000,3000,4000};
   
   
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(irNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  while (robot->step(TIME_STEP) != -1) {
    
    bool exit = true;
    
    for (int i = 0; i < 8; i++) {
        if (ir[i]->getValue() <150 ){
          ir_readings[i]=0;
          
          }
        else{
          ir_readings[i]=1;
          exit = false;
        }  
    } 
    
    if (exit){break;}
   
    double error = 0.0;
    for (int j =0; j<8;j++){
      error=error+ir_readings[j]*coefficient[j];
    }
    
    double P=kp*error;
    double I= (ki*error)+integral;
    double D= kd*(error-prevoious_error);
    
    
    double correction = (P+I+D)/1000;
    double left_motor = normal_speed + correction;
    double right_motor = normal_speed - correction;
    
    if (left_motor <0.0) {left_motor=0.0;}
    if (left_motor >10.0) {left_motor=10.0;}
    if (right_motor<0.0) {right_motor=0.0;}
    if (right_motor>10.0) {right_motor=10.0;}
    
    
    motors[0]->setVelocity(left_motor);
    motors[1]->setVelocity(right_motor);
    
    prevoious_error = error;
    integral = I;  
  }

}



void solve_maze(){

  double kp=10;
  double kd=0.08;
  double ki=0.001;
  double prevoious_error = 0.0;
  double integral = 0.0;
  
  
  for (int i = 0; i < 3; i++) {
      us[i] = robot->getDistanceSensor(usNames[i]);
      us[i]->enable(TIME_STEP);
    }
  
    while (robot->step(TIME_STEP) != -1) {
    
      double us_left = us[0]->getValue();
      double us_front = us[1]->getValue();
      double us_right = us[2]->getValue();
       
     //std::cout<< "left : "<<us_left<<" right : "<< us_right-40 <<std::endl;
      
      if ((us_front>240 && us_left<150) && ((us_right-40)-us_left)<100 ){
      
        double error = (us_left- (us_right-40) )/4 + (98 -us_right)/12;
        double P=kp*error;
        double I= (ki*error)+integral;
        double D= kd*(error-prevoious_error);
   
        double correction = (P+I+D)/20;
        double left_motor = normal_speed + correction;
        double right_motor = normal_speed - correction;
        
        if (left_motor <0.0) {left_motor=0.0;}
        if (left_motor >10.0) {left_motor=10.0;}
        if (right_motor<0.0) {right_motor=0.0;}
        if (right_motor>10.0) {right_motor=10.0;}
         
        motors[0]->setVelocity(left_motor);
        motors[1]->setVelocity(right_motor);
        
        prevoious_error = error;
        integral = I;
      }
      
      else if (us_front>240 && us_left>150){
        prevoious_error = 0.0;
        integral = 0.0;
        turn_left();
        while (robot->step(TIME_STEP) != -1){
        if (us[0]->getValue()<150){break;}
        left_motor = normal_speed;
        right_motor = normal_speed;
        motors[0]->setVelocity(left_motor);
        motors[1]->setVelocity(right_motor);
        }  
        }
        
      else if (us_front<240 && us_left>150){
        prevoious_error = 0.0;
        integral = 0.0;
        turn_left();
        while (robot->step(TIME_STEP) != -1){
        if (us[0]->getValue()<150){break;}
        left_motor = normal_speed;
        right_motor = normal_speed;
        motors[0]->setVelocity(left_motor);
        motors[1]->setVelocity(right_motor);
        }  
      }
      
      else if (us_front<150 && us_left<200){
      prevoious_error = 0.0;
      integral = 0.0;
      
      turn_right();
      if (us[1]->getValue()<200) {
        turn_right();
      }
        
      }  
    } 
}
