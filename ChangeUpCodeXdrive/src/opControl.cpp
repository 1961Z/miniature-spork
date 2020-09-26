#include "vex.h"

#include "opControl.h"

#include "autonFunctions.h"

using namespace vex;

/*-----------------------------------------------------------------------------*/
/** @brief     Base Control */
/*-----------------------------------------------------------------------------*/
int joyStickControl() {
  while (true) {
    double front_left = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct));
    double back_left = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct));
    double front_right = (double)(Controller1.Axis3.position(pct) - Controller1.Axis4.position(pct));
    double back_right = (double)(Controller1.Axis3.position(pct) + Controller1.Axis4.position(pct));

    double max_raw_sum = (double)(abs(Controller1.Axis3.position(pct)) + abs(Controller1.Axis4.position(pct)));

    double max_XYstick_value = (double)(std::max(abs(Controller1.Axis3.position(pct)), abs(Controller1.Axis4.position(pct))));

    if (max_raw_sum != 0) {
      front_left = front_left / max_raw_sum * max_XYstick_value;
      back_left = back_left / max_raw_sum * max_XYstick_value;
      front_right = front_right / max_raw_sum * max_XYstick_value;
      back_right = back_right / max_raw_sum * max_XYstick_value;
    }

    front_left = front_left + Controller1.Axis1.position(pct);
    back_left = back_left + Controller1.Axis1.position(pct);
    front_right = front_right - Controller1.Axis1.position(pct);
    back_right = back_right - Controller1.Axis1.position(pct);

    max_raw_sum = std::max(fabs(front_left), std::max(fabs(back_left), std::max(fabs(front_right), std::max(fabs(back_right), 100.0))));

    front_left = front_left / max_raw_sum * 100.0;
    back_left = back_left / max_raw_sum * 100.0;
    front_right = front_right / max_raw_sum * 100.0;
    back_right = back_right / max_raw_sum * 100.0;

    front_L.spin(forward, front_left , velocityUnits::pct);
    front_R.spin(forward, front_right , velocityUnits::pct);
    back_L.spin(forward, back_left , velocityUnits::pct);
    back_R.spin(forward, back_right , velocityUnits::pct);

    task::sleep(10);
  }
}



void outtakeAll() {
  if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
    conveyor_L.spin(fwd, -100, pct);
    conveyor_R.spin(fwd, -100, pct);
    intake_L.spin(fwd, -100, pct);
    intake_R.spin(fwd, -100, pct);
  }
  else{ 
    conveyor_L.stop();
    conveyor_R.stop();
    intake_L.stop();
    intake_R.stop();
  }
}

int conveyorToggle() {
  while (true) {

    if (Controller1.ButtonL2.pressing()) {
      task f = task(outtake1Ball);
    } 
    else if (Controller1.ButtonA.pressing()) {
      task m = task(outtake3Ball);
    }
    else if(Controller1.ButtonB.pressing()){
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
      //task::stop(outtake1Ball);
      task::resume(primeShoot);
    }
    task::sleep(10);
  }
}
