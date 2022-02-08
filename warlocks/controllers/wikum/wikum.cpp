// File:          motor_testing.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

#define TIME_STEP 64

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
    Motor *motor1 = robot->getMotor("left_motor");
    Motor *motor2 = robot->getMotor("right_motor");
    Motor *v_linear = robot->getMotor("v_linear_motor");
    Motor *hr_linear = robot->getMotor("hr_linear_motor");
    Motor *hl_linear = robot->getMotor("hl_linear_motor");
    
    motor1->setPosition(INFINITY);
    motor2->setPosition(INFINITY);
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    //motor1->setVelocity(2.0);
    //motor2->setVelocity(2.0);
    // Process sensor data here.
    v_linear->setPosition(0.05);
    hl_linear->setPosition(0.025);
    hr_linear->setPosition(-0.025);
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}