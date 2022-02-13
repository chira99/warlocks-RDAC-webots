#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 16
<<<<<<< HEAD
#define normal_speed 8
=======
#define normal_speed 2
>>>>>>> def7653296e5b36a13b3688bc1bbe47af3c24129

using namespace webots;

Robot *robot = new Robot();
Motor *motors[2] = {robot->getMotor("left_motor"), robot->getMotor("right_motor")};
Motor *linear_motors[3]  ={robot->getMotor("v_linear_motor"),robot->getMotor("hr_linear_motor"),robot->getMotor("hl_linear_motor")};
DistanceSensor *us[3];
DistanceSensor *ir[8];
Camera *cm = robot->getCamera("cam");

char irNames[8][5]  = {"ir1","ir2","ir3","ir4","ir5","ir6","ir7","ir8"};
char usNames[3][10] = {"us_left","us_front","us_right"};
double left_motor = 0.0;
double right_motor= 0.0;

void delay(float t);
void turn_left();
void turn_right();
void turn_back();
void follow_continous_line();
void follow_dashed_line();
void solve_maze();

void Arm_horizontal(float d);
void Arm_verticle(float d);

int main(int argc, char **argv) {
  Arm_horizontal(0.04);
  Arm_verticle(0.1);
 
  for (int i = 0; i < 2; i++) {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }
 
  cm->enable(TIME_STEP);
  
  while (robot->step(TIME_STEP) != -1) {
<<<<<<< HEAD
    //follow_continous_line();
    //delay(500);
    //solve_maze(); 
    follow_dashed_line();
=======
    follow_continous_line();
    //delay(500);
    //solve_maze(); 
       
>>>>>>> def7653296e5b36a13b3688bc1bbe47af3c24129
  }
}  
//////////////////////////////////////////////////////// 
void follow_dashed_line(){

  int ir_readings[8];
  double kp = 10;
  double kd = 0.1;
  double ki = 0.2;
  
  double previous_error = 0.0;
  double integral = 0.0;

  int coefficient[8] = {-4000,-3000,-2000,-1000,1000,2000,3000,4000};
   
   
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(irNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  bool allBlack_before = false;
  
  while (robot->step(TIME_STEP) != -1) {
<<<<<<< HEAD
  
    bool allBlack = true;
   
=======
    
    //bool exit = true;
    
>>>>>>> def7653296e5b36a13b3688bc1bbe47af3c24129
    for (int i = 0; i < 8; i++) {
        if (ir[i]->getValue() < 150){
          ir_readings[i]=0;
          }
        else{
          ir_readings[i]=1;
<<<<<<< HEAD
          allBlack = false;
=======
          //exit = false;
>>>>>>> def7653296e5b36a13b3688bc1bbe47af3c24129
        }  
    }
    //reset integral at each dash line
    if (allBlack == false && allBlack_before == true){
    integral = 0.0;
    }
    
    if (allBlack == true){
    allBlack_before = true;
    }
    
    
    //Print IR Readings
    std::cout <<"{\n";
    for(int i=0;i<8;i++)
    {
      std::cout << ir[i]->getValue() <<" ";
    } 
    std::cout <<"}\n";
    
<<<<<<< HEAD

=======
    //if (exit){break;}
   
>>>>>>> def7653296e5b36a13b3688bc1bbe47af3c24129
    double error = 0.0;
    for (int j=0; j<8;j++){
      error += ir_readings[j]*coefficient[j];
    }
    
    double P = kp*error;
    double I =(ki*error)+integral;
    double D = kd*(error-previous_error);
    
    std::cout << P << " " << I << " " << D << "\n";
    
    double correction  = (P+I+D)/1000;
    double left_motor  = normal_speed + correction;
    double right_motor = normal_speed - correction;
    
    if (left_motor < 0.0)  {left_motor=0.0;}
    if (left_motor > 10.0) {left_motor=10.0;}
    if (right_motor< 0.0)  {right_motor=0.0;}
    if (right_motor> 10.0) {right_motor=10.0;}
    
    motors[0]->setVelocity(left_motor);
    motors[1]->setVelocity(right_motor);
    
    previous_error = error;
    integral = I;  
  }

}


void follow_continous_line(){

  int ir_readings[8];
  double kp = 3;
  double kd = 0.2;
  double ki = 0.01;
  double previous_error = 0.0;
  double integral = 0.0;
  int coefficient[8] = {-4000,-3000,-2000,-1000,1000,2000,3000,4000};
   
   
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(irNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  while (robot->step(TIME_STEP) != -1) {
    
    bool exit = true;
    
    for (int i = 0; i < 8; i++) {
        if (ir[i]->getValue() < 150){
          ir_readings[i]=0;
          }
        else{
          ir_readings[i]=1;
          exit = false;
        }  
    } 
    
    
    if (exit){break;}
   
    double error = 0.0;
    for (int j=0; j<8;j++){
      error=error+ir_readings[j]*coefficient[j];
    }
    
    double P= kp*error;
    double I= (ki*error)+integral;
    double D= kd*(error-previous_error);
    
    
    double correction = (P+I+D)/1000;
    double left_motor = normal_speed + correction;
    double right_motor = normal_speed - correction;
    
    if (left_motor < 0.0)  {left_motor=0.0;}
    if (left_motor > 10.0) {left_motor=10.0;}
    if (right_motor< 0.0)  {right_motor=0.0;}
    if (right_motor> 10.0) {right_motor=10.0;}
    
    
    motors[0]->setVelocity(left_motor);
    motors[1]->setVelocity(right_motor);
    
    previous_error = error;
    integral = I;  
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

void solve_maze(){

  double kp=10;
  double kd=0.08;
  double ki=0.001;
  double previous_error = 0.0;
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
        double D= kd*(error-previous_error);
   
        double correction = (P+I+D)/20;
        double left_motor = normal_speed + correction;
        double right_motor = normal_speed - correction;
        
        if (left_motor <0.0) {left_motor=0.0;}
        if (left_motor >10.0) {left_motor=10.0;}
        if (right_motor<0.0) {right_motor=0.0;}
        if (right_motor>10.0) {right_motor=10.0;}
         
        motors[0]->setVelocity(left_motor);
        motors[1]->setVelocity(right_motor);
        
        previous_error = error;
        integral = I;
      }
      
      else if (us_front>240 && us_left>150){
        previous_error = 0.0;
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
        previous_error = 0.0;
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
      previous_error = 0.0;
      integral = 0.0;
      
      turn_right();
      if (us[1]->getValue()<200) {
        turn_right();
      }
        
      }  
    } 
}
///////////////////////////////////////////////////////////////////
void Arm_horizontal(float d)
{
  linear_motors[2]->setPosition(d);
  linear_motors[1]->setPosition(-1*d);
}  

void Arm_verticle(float d)
{
  linear_motors[0]->setPosition(d);
} 