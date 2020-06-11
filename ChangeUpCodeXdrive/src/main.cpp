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
  moveForwardWalk(100, 100, 0, 0.5);   
  //moveForward(70, 24);
  //rotatePID(90, 100);
  //strafeWalk(-24, 100, 90, 0.5);
  //ObjectLooker(1, 50);
  /*rotatePID(145, 90);
  strafeWhileTurning(10, 10);
  rotatePID(225,90);*/
  strafeWalk(100, 100, 0, 0.5);

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
  back_L.stop(coast);
  back_R.stop(coast);
  front_L.stop(coast);
  front_R.stop(coast);
  front_L.setVelocity(0, pct);
  front_R.setVelocity(0, pct);
  back_L.setVelocity(0, pct);
  back_R.setVelocity(0, pct);
  inertial_Up.calibrate();
  while (inertial_Up.isCalibrating()) {
    wait(100, msec);
  }
  inertial_Down.calibrate();
  while (inertial_Down.isCalibrating()) {
    wait(100, msec);
  }
  task q = task(joyStickControl);
  task z = task(autoAlignWithGoal);
  task a = task(intakeToggle);
  task x = task(conveyorToggle);

  while (1) {

    /*double rotationRight = pow(front_R.rotation(rev), 2); 
    double rotationLeft = pow(front_L.rotation(rev), 2);
    double rotationRightBack = pow(back_R.rotation(rev), 2);
    double rotationLeftBack = pow(back_L.rotation(rev), 2);
    double distanceTraveledForward = sqrt(rotationLeft + rotationRight);
    double distanceTraveledStrafeLeft = sqrt( rotationLeft + rotationLeftBack);
    double distanceTraveledStrafeRight = sqrt(rotationRight + rotationRightBack);*/
    /*printf("encoder forward %f\n", distanceTraveledForward);
    printf("encoder strafeLeft %f\n", distanceTraveledStrafeLeft);
    printf("encoder strafeRight %f\n", distanceTraveledStrafeRight);*/

    Brain.Screen.printAt(1, 20, "Encoder rotations: %f left", inertial_Down.rotation(deg));
    Brain.Screen.printAt(1, 40, "Encoder rotations: %f leftBack", inertial_Up.rotation(deg));
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