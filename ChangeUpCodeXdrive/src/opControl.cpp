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

    //Find the largest possible sum of X and Y
    double max_raw_sum = (double)(abs(Controller1.Axis3.position(pct)) + abs(Controller1.Axis4.position(pct)));

    //Find the largest joystick value
    double max_XYstick_value = (double)(std::max(abs(Controller1.Axis3.position(pct)), abs(Controller1.Axis4.position(pct))));

    //The largest sum will be scaled down to the largest joystick value, and the others will be
    //scaled by the same amount to preserve directionality
    if (max_raw_sum != 0) {
      front_left = front_left / max_raw_sum * max_XYstick_value;
      back_left = back_left / max_raw_sum * max_XYstick_value;
      front_right = front_right / max_raw_sum * max_XYstick_value;
      back_right = back_right / max_raw_sum * max_XYstick_value;
    }

    //Now to consider rotation
    //Naively add the rotational axis
    front_left = front_left + Controller1.Axis1.position(pct);
    back_left = back_left + Controller1.Axis1.position(pct);
    front_right = front_right - Controller1.Axis1.position(pct);
    back_right = back_right - Controller1.Axis1.position(pct);

    //What is the largest sum, or is 100 larger?
    max_raw_sum = std::max(fabs(front_left), std::max(fabs(back_left), std::max(fabs(front_right), std::max(fabs(back_right), 100.0))));

    //Scale everything down by the factor that makes the largest only 100, if it was over
    front_left = front_left / max_raw_sum * 100.0;
    back_left = back_left / max_raw_sum * 100.0;
    front_right = front_right / max_raw_sum * 100.0;
    back_right = back_right / max_raw_sum * 100.0;

    //Write the manipulated values out to the motors

    front_L.spin(forward, front_left * 0.12, voltageUnits::volt);
    front_R.spin(forward, front_right * 0.12, voltageUnits::volt);
    back_L.spin(forward, back_left * 0.12, voltageUnits::volt);
    back_R.spin(forward, back_right * 0.12, voltageUnits::volt);

    task::sleep(10);
  }
}

int conveyorToggle() {
  while (true) {

    if (Controller1.ButtonL1.pressing()) {
      task::stop(primeShoot);
      task::stop(intakeToggle);
      //printf("I come here too");
      task f = task(outtake1Ball);
      task::resume(intakeToggle);
    } 
    task::sleep(10);
  }
}

int autoAlignWithGoal() {
  while (true) {
   primShooterWithLimit();
  }
  task::sleep(10);
 }


int primeTheConveyor(){
  while(true){
    if(Controller1.ButtonX.pressing()){
      task::stop(outtake1Ball);
      task::resume(primeShoot);
    }
    task::sleep(10);
  }
}
