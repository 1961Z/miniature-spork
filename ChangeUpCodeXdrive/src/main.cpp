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
  inertialCalibration();

}

void autonomous(void) { 
  
  inertialCalibration();
  task fakeSpaceMan = task(update);  
  homeRowAuton(); 

}

void usercontrol(void) {

  coastDrive();
  task s = task(joyStickControl);
  task e = task(intakeToggle);
  task x = task(conveyorToggle);
  task y = task(BallCount);
  task z = task(toggle);
  task fakeSpaceMan = task(update);

  while (1) {

    outtakeAll(); 
    printf("Theta: %f\n", average_inertial());
    printf("leftfront: %f\n", front_L.velocity(pct));
    printf("leftback: %f\n", back_L.velocity(pct));
    printf("rightfront: %f\n", front_R.velocity(pct));
    printf("rightback: %f\n", back_R.velocity(pct));
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
