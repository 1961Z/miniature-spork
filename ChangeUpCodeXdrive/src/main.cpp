/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       1961Z                                                     */
/*    Created:      May 2020 - May 2021                                       */
/*    Description:  1961Z ChangeUp Code                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "opControl.h"

#include "tracking.h"

#include "autonFunctions.h"

#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  
  vexcodeInit();
  preAuton();

}

void autonomous(void) { 
  
  task fakeSpaceMan = task(update);  
  homeRowAuton();
  //move_to_target_sync(36, 24, deg_to_rad(90), false, 127, false);
  //skills(); 
  //rotatePID(30, 90);
  //forwardWhileRotating30to90(0, 0, 0, 60);
}

void usercontrol(void) {
  
  //inertialCalibration();
  coastDrive();
  task s = task(joyStickControl);
  task e = task(intakeToggle);
  task x = task(conveyorToggle);
  task y = task(BallCount);
  task z = task(primeTheConveyor);
  task fakeSpaceMan = task(update);

  while (1) {

    //outtakeAll(); 
    double driftLeftError = (front_R.rotation(rev) + back_L.rotation(rev));
    double driftRightError = (front_L.rotation(rev) + back_R.rotation(rev));
    double combinedDriftError = ((driftLeftError - driftRightError));
    /*double rotationLeft = pow(front_L.rotation(rev), 2);
    double rotationLeftBack = pow(back_L.rotation(rev), 2);
    combinedDriftError = sqrt(rotationLeft + rotationLeftBack);*/

    double combinedDriftErrorMultiply = combinedDriftError * (-3.14159); 

    double horizontalTrackerError = (horizontalTracker.rotation(rev) * (3.14 * 2.85) * 1);
      
    double pogChamp = ((horizontalTrackerError + combinedDriftErrorMultiply) * 0.5); //* (circumference);
    
    printf("Drift: %f\n", combinedDriftErrorMultiply);
    printf("Horizontal: %f\n", horizontalTrackerError);
    printf("pogChamp: %f\n", pogChamp);
    //printf("leftfront: %f\n", back_L.rotation(rev));
    /*printf("leftback: %f\n", back_L.velocity(pct));
    printf("rightfront: %f\n", front_R.velocity(pct));
    printf("rightback: %f\n", back_R.velocity(pct));*/
    //printf("Light Sensor Middle %ld\n", LineTrackerMiddle.reflectivity());
    //printf("Light Sensor Intake %ld\n", LineTrackerIntake.reflectivity());
    //printf("Light Sensor Top %ld\n", LineTrackerTop.reflectivity());
    wait(10, msec); 

  }
}

int main() {
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
  