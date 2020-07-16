/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFotward          motor         1
// LeftBack             motor         2
// RightForward         motor         3
// RightBack            motor         4
// Controller1          controller
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "opControl.h"

#include "autonFunctions.h"

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
 inertial_Up.calibrate();
   while (inertial_Up.isCalibrating()) {
     wait(100, msec);
  }
  inertial_Down.calibrate();
  while (inertial_Down.isCalibrating()) {
   wait(100, msec);
  }
  task y = task(BallCount);
  task bitch = task(intakeOn);
  moveForwardWalk(16, 80, 0, 0.6, 50);
  rotatePID(30, 90);
  task::stop(bitch);
  intake_R.stop(brake);
  intake_L.stop(brake);
  task nword = task(primeShoot);
  moveForwardWalk(6, 80, 30, 0.6, 50);
  outtake1BallAuton();
  moveForwardWalk(-38,80, 30, 0.6, 50);
  strafeWhileTurning(10, 24);
  moveForwardWalk(6, 80, 90, 0.6, 50);
  outtake1BallAuton();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  front_L.stop(coast);
  front_R.stop(coast);
  back_L.stop(coast);
  back_R.stop(coast);
  /*inertial_Up.calibrate();
  while (inertial_Up.isCalibrating()) {
    wait(100, msec);
  }
  inertial_Down.calibrate();
  while (inertial_Down.isCalibrating()) {
    wait(100, msec);
  }*/
  task s = task(joyStickControl);
  task e = task(intakeToggle);
  task x = task(conveyorToggle);
  task y = task(BallCount);
  task z = task(toggle);
  while (1) {
    testForPrabu(); 
    /*double rotationRight = pow(front_R.rotation(rev), 2);
    double rotationLeft = pow(front_L.rotation(rev), 2);
    double rotationRightBack = pow(back_R.rotation(rev), 2);
    double rotationLeftBack = pow(back_L.rotation(rev), 2);
    double distanceTraveledForward = sqrt(rotationLeft + rotationRight);
    double distanceTraveledStrafeLeft = sqrt( rotationLeft + rotationLeftBack);
    double distanceTraveledStrafeRight = sqrt(rotationRight + rotationRightBack);*/
    printf("encoder forward %f\n", verticalTracker.rotation(degrees));
    printf("encoder strafeLeft %f\n", horizontalTracker.rotation(degrees));
    //printf("encoder strafeRight %f\n", distanceTraveledStrafeRight);

    //printf("Light Sensor Middle %ld\n", LineTrackerMiddle.reflectivity());
    //printf("Light Sensor Intake %ld\n", LineTrackerIntake.reflectivity());
    // printf("Light Sensor Top %ld\n", LineTrackerTop.reflectivity());
    // Brain.Screen.printAt(1, 20, "Encoder rotations: %f left", inertial_Down.rotation(deg));
    // Brain.Screen.printAt(1, 40, "Encoder rotations: %f leftBack", inertial_Up.rotation(deg));
    wait(10, msec); // Sleep the task for a short amount of time to
    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
