#include "vex.h"

#include "opControl.h"

#include "autonFunctions.h"

using namespace vex;

double leftDriveSpeed, rightDriveSpeed, leftDriveCalculation, rightDriveCalculation;

/*-----------------------------------------------------------------------------*/
/** @brief     Base Control */
/*-----------------------------------------------------------------------------*/
int joyStickControl() {
  while (true) {
    leftDriveCalculation = (Controller1.Axis3.position() + (Controller1.Axis1.position()));
    rightDriveCalculation = (Controller1.Axis3.position() - (Controller1.Axis1.position()));
    if(fabs(leftDriveCalculation) >= 40){
      leftDriveSpeed = leftDriveCalculation; 
    }
    else {
      leftDriveSpeed = (leftDriveCalculation * 0.5);
    }

    if(fabs(rightDriveCalculation) >= 40){
      rightDriveSpeed = rightDriveCalculation; 
    }
    else {
      rightDriveSpeed = (rightDriveCalculation * 0.5);
    }
    leftDriveSpeed = sgn(leftDriveCalculation) * ( 0.01 *(pow(leftDriveCalculation, 2))); 
    rightDriveSpeed = sgn(rightDriveCalculation) * ( 0.01 *(pow(rightDriveCalculation, 2))); 

    //printf("leftDrive %ld\n", Controller1.Axis3.position());
    //printf("rightDrive %ld\n", Controller1.Axis1.position());
    
    //leftDrive.spin (fwd, (Controller1.Axis3.position() + (Controller1.Axis1.position())), pct);
    //rightDrive.spin(fwd, (Controller1.Axis3.position() - (Controller1.Axis1.position())), pct);
    leftDrive.spin(fwd, leftDriveSpeed, pct);
    rightDrive.spin(fwd, rightDriveSpeed, pct);
    task::sleep(10);
  }
}

int conveyorToggle() {
  while (true) {
    if (Controller1.ButtonLeft.pressing()){
      task m = task(outtake3Ball);
    }
    else if (Controller1.ButtonL1.pressing()) {
      task f = task(outtake1Ball);
    } 
    else if(Controller1.ButtonL2.pressing()){
      task q = task(outtake2Ball);
    }
    else{
      //nothing needed in else but it just makes the code look a bit cleaner
    }
    task::sleep(10);
  }
}

int autoAlignWithGoal() {
  while (true) {
   primShooterWithLimit();
   task::sleep(10);
  }
 }
 
 bool hit = false; 
 bool switchMode = false;

 int toggle() {
 static bool lowTower = false; //coolio

  while (true) {

    if (Controller1.ButtonL1.pressing() && hit == false) {

      hit = 1;
      lowTower = !lowTower;
      switchMode = !switchMode;

      if (lowTower == true) {
       task::stop(goBackDown);
       task::resume(primeShoot);
      }

      else {
        task::stop(primeShoot);
        task::resume(goBackDown);
      }
    }

    if (!Controller1.ButtonL1.pressing() && hit == 1) {
      hit = 0; 
      int timerCountDown = 0; 
      while (timerCountDown < 1000) {
      task::sleep(10);
      timerCountDown += 10;
      }
      task::stop(goBackDown); 
    }
    task::sleep(10);
  }
}



int primeTheConveyor(){
  while(true){
    if(Controller1.ButtonL1.pressing()){
      indexer.spin(fwd, 127, volt);
    }
    else if(Controller1.ButtonL2.pressing()){
      indexer.spin(reverse, 127, volt);
    }
    else{
      indexer.stop(coast);
    }
    task::sleep(10);
  }
}
