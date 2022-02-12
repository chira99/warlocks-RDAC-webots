// File:          chathuni.cpp
// Date:          2022/02/10
// Description:   Code for navigating in mosaic floor upto the keyholes
// Author:        Chathuni
// Modifications:

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Display.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define TIME_STEP 16
#define MOTOR_SPEED 3
#define MtoC_ENCODER 35
#define MtoY_ENCODER 50
#define HALF_CIRCLE 31.2
#define WHEEL_RADIUS 35   // radius = 35 mm
#define ENCODER_UNIT 3.5     // (2*3.14*3.5/6.28)
#define CAM_HEIGHT 64
#define CAM_WIDTH 64

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
using namespace cv;

  // Declaring Objects
Robot *robot = new Robot();
Motor *left_motor = robot->getMotor("left_motor");
Motor *right_motor = robot->getMotor("right_motor");
Camera *camera = robot->getCamera("cam");
PositionSensor *left_encoder = robot->getPositionSensor("left_encoder");
PositionSensor *right_encoder = robot->getPositionSensor("right_encoder");
DistanceSensor *ToF = robot->getDistanceSensor("tof");

void pick_first_obj(void);
void turn_clockwise(int angle);
void go_distance(int dist);
void go_rev_distance(int dist);
float turn_to_object(int color,int threshold);
int obj_identify(int color);

float first_object_turn_angle;
float first_object_distance;
float second_object_turn_angle;
float second_object_distance;

static Scalar lMargin[2] = {Scalar(20,0,0),Scalar(20,0,0)};    // Yellow = 0, White = 1
static Scalar uMargin[2] = {Scalar(50,255,255),Scalar(35,255,255)};

int main(int argc, char **argv) {

  // Initializing Motors
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  // camera->disable();

  turn_clockwise(HALF_CIRCLE);
  pick_first_obj();

  // // Main loop:
  // while (robot->step(TIME_STEP) != -1) {

    
  // }
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

void go_distance(int dist){
  //** This function drives the robot the given distance forward **//

  // Variables for storing values
  float encoder_values[2] = {0,0};

  // Initializing Encoders
  left_encoder->enable(TIME_STEP);
  right_encoder->enable(TIME_STEP);

  // Starting the motors
  left_motor->setVelocity(-MOTOR_SPEED);
  right_motor->setVelocity(-MOTOR_SPEED);

  float left_dis = 0, right_dis = 0, distance = 0;

  float left_start = left_encoder->getValue();
  float right_start = right_encoder->getValue();

  while ((robot->step(TIME_STEP) != -1) && (distance < dist)){

    encoder_values[0] = left_encoder->getValue();
    encoder_values[1] = right_encoder->getValue();

    left_dis = (encoder_values[0] - left_start) * -ENCODER_UNIT;
    right_dis = (encoder_values[1] - right_start) * -ENCODER_UNIT;
    distance = (left_dis+right_dis)/2;

    // cout << "Distance = " << distance << endl;
  }

  left_motor->setVelocity(0);
  right_motor->setVelocity(0);

  left_encoder->disable();
  right_encoder->disable();
}

void go_rev_distance(int dist){
  //** This function drives the robot the given distance forward **//

  // Variables for storing values
  float encoder_values[2] = {0,0};

  // Initializing Encoders
  left_encoder->enable(TIME_STEP);
  right_encoder->enable(TIME_STEP);

  // Starting the motors
  left_motor->setVelocity(MOTOR_SPEED);
  right_motor->setVelocity(MOTOR_SPEED);

  float left_dis = 0, right_dis = 0, distance = 0;

  float left_start = left_encoder->getValue();
  float right_start = right_encoder->getValue();

  while ((robot->step(TIME_STEP) != -1) && (distance > -dist)){

    encoder_values[0] = left_encoder->getValue();
    encoder_values[1] = right_encoder->getValue();

    left_dis = (encoder_values[0] - left_start) * -ENCODER_UNIT;
    right_dis = (encoder_values[1] - right_start) * -ENCODER_UNIT;
    distance = (left_dis+right_dis)/2;

    // cout << "Distance = " << distance << endl;
  }

  left_motor->setVelocity(0);
  right_motor->setVelocity(0);

  left_encoder->disable();
  right_encoder->disable();
}

float turn_to_object(int color,int threshold){ // Yellow = 0, White = 1
  //** This function turns the robot until an object of the given color is straightly ahead **//

  // Variables for storing values
  float encoder_values[2] = {0,0};
  float turn_angle = 0;

  // Initializing Encoders
  left_encoder->enable(TIME_STEP);
  right_encoder->enable(TIME_STEP);

  // float left_start = left_encoder->getValue();
  // float right_start = right_encoder->getValue();
  Mat img = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat img_hsv = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat filter_out = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat filtered = Mat(Size(CAM_WIDTH, CAM_HEIGHT), CV_8UC4);
  const unsigned char *cam_img;
  // static unsigned char *processed_image;
  camera->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){

    left_motor->setVelocity(0);
    right_motor->setVelocity(0);

    cam_img = camera->getImage();
    img.data = (uchar *)cam_img;

    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv,lMargin[color],uMargin[color],filter_out);

    erode(filter_out, filter_out, getStructuringElement(MORPH_ELLIPSE, Size(threshold, threshold)));
    dilate(filter_out, filter_out, getStructuringElement(MORPH_ELLIPSE, Size(threshold, threshold)));

    Moments obj_moment = moments(filter_out);
    // double dM01 = obj_moment.m01;
    double dM10 = obj_moment.m10;
    double dArea = obj_moment.m00;

    int posX = dM10/dArea;

    // cout << "X = " << posX << endl;

    if ((posX <= CAM_WIDTH/2+1) && (posX >= CAM_WIDTH/2-1)){

      encoder_values[0] = left_encoder->getValue();
      encoder_values[1] = right_encoder->getValue();

      turn_angle = (encoder_values[0]-encoder_values[1])*ENCODER_UNIT/2;

      // cout << "Should turn " << turn_angle << endl;

      break;
    }

    left_motor->setVelocity(-MOTOR_SPEED*0.3);
    right_motor->setVelocity(MOTOR_SPEED*0.3);

  }

  // camera->disable();

  left_encoder->disable();
  right_encoder->disable();

  return turn_angle;

}

int obj_identify(int color){

  Mat img = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat img_hsv = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat filter_out = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat noise_removed = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat thresholded = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  Mat canny = Mat(Size(CAM_WIDTH,CAM_HEIGHT),CV_8UC4);
  const unsigned char *cam_img;
  vector<vector<Point>> contours;
  vector<Vec4i> heirarchy;
  vector<vector<Point>> conPoly;
  int n_points = 0;
  int area_threshold = 10;
  int area = 0;

  camera->enable(TIME_STEP);

  cam_img = camera->getImage();
  img.data = (uchar *)cam_img;

  cvtColor(img, img_hsv, COLOR_BGR2HSV);
  inRange(img_hsv,lMargin[color],uMargin[color],filter_out);

  bilateralFilter(filter_out,noise_removed,9,75,75);
  threshold(noise_removed,thresholded,0,255,THRESH_OTSU);

  erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
  dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

  findContours(thresholded, contours, heirarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  conPoly.resize(contours.size());

  // cout << "No. of Contours " << contours.size() <<endl;
  for (size_t i = 0; i < contours.size(); i++){
    area = contourArea(contours[i]);

    cout << "Contour " << i+1 << " Area: " << area << endl;

    if (area > area_threshold)
    {
      float peri = arcLength(contours[i], true);
      approxPolyDP(Mat(contours[i]),conPoly[i], 0.01*peri, true);
      n_points = (int)conPoly[i].size();

      cout << "No. of points: " << n_points << endl;
    }
  }

  camera->disable();
  if (/*area > 300 &&*/ n_points < 10){
    cout << "Identified Box" << endl;
    return 1;  // For Box
  }
  else if (/*area <= 300 &&*/n_points >= 10){
    cout << "Identified Cylinder" << endl;
    return 0;   // For Cylinder
  }
  else{
    return -1;
  }
}

void pick_first_obj(void){
  //** This function moves the robot from the end of the maze upto picking up the first object **//

  go_distance(MtoC_ENCODER);
  first_object_turn_angle = turn_to_object(0,2);  // Turn towards a yellow object, threshold is high to detect the larger object when both are visible

  // cout <<  "Will turn " << first_object_turn_angle << endl;

  ToF->enable(TIME_STEP);
  double distance = ToF->getValue()/5;
  cout << "Distance to Object: " << distance << " in cm" << endl;
  first_object_distance = distance - 7;

  go_distance(distance-10);

  int obj_type = -1;

  while (obj_type == -1){
    obj_type = obj_identify(0);
  }

  go_rev_distance(first_object_distance-4);
  turn_clockwise(first_object_turn_angle*1.34);
  go_rev_distance(MtoC_ENCODER);

}

void turn_clockwise(int angle){
  /* This function turn sthe robot 180 degreen around */

  // Variables for storing values
  float encoder_values[2] = {0,0};

  // Initializing Encoders
  left_encoder->enable(TIME_STEP);
  right_encoder->enable(TIME_STEP);

  // Starting the motors
  left_motor->setVelocity(MOTOR_SPEED*0.5);
  right_motor->setVelocity(-MOTOR_SPEED*0.5);

  float left_dis = 0, right_dis = 0, distance = 0;

  while ((robot->step(TIME_STEP) != -1) && (distance < angle)){

    encoder_values[0] = left_encoder->getValue();
    encoder_values[1] = right_encoder->getValue();

    left_dis = (encoder_values[0]) * ENCODER_UNIT;
    right_dis = (-(encoder_values[1])) * ENCODER_UNIT;
    distance = (left_dis+right_dis)/2;

    // cout << "Left Encoder = " << encoder_values[0] << endl;
    // cout << "Right Encoder = " << encoder_values[1] << endl;
    // cout << "Left Distance = " << left_dis << endl;
    // cout << "Right Distance = " << right_dis << endl;
    // cout << "Distance = " << distance << endl;
  }

  left_motor->setVelocity(0);
  right_motor->setVelocity(0);

  left_encoder->disable();
  right_encoder->disable();

}