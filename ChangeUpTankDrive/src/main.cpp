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
  inertialCalibration();
  testAuton();
}

void usercontrol(void) {
  indexer.spin(fwd, 100, pct);
  task::sleep(500);
  indexer.stop(brake);
  coastDrive();
  //inertialCalibration();
  task s = task(joyStickControl);
  task e = task(intakeToggle);
  task x = task(conveyorToggle);
  task y = task(BallCount);

  while (1) {

    //outtakeAll(); 

    if(leftDrive.temperature(pct) > 50){
      Controller1.rumble("----");
    }
    
    //printf("Base temp: %f\n", front_L.temperature(pct));
    ///printf("Base temp: %f\n", back_L.temperature(pct));
    //float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
    //printf("heading %f\n", robotDirection);
    float position = (-(leftTracker.rotation(rev)) + (rightTracker.rotation(rev))) / 2;
    printf("position %f\n", position);
    //printf("left %f\n", leftTracker.rotation(rev));
    //printf("right %f\n", rightTracker.rotation(rev));
    //printf("Horizontal Tracker %f\n", back_L.rotation(rev));
    //printf("leftfront: %f\n", back_L.rotation(rev));
    /*printf("leftback: %f\n", back_L.velocity(pct));
    printf("rightfront: %f\n", front_R.velocity(pct));
    printf("rightback: %f\n", back_R.velocity(pct));*/
    //printf("Light Sensor Middle %ld\n", goalChecker.reflectivity());
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
  