#include "vex.h"

#include "visionSensorCode.h"

#include "autonFunctions.h"

using namespace vex;

void resetFunction() {
  back_L.resetRotation();
  back_R.resetRotation();
  front_L.resetRotation();
  front_R.resetRotation();
}

void visionRGB() {
  double r = 0;
  double g = 0;
  double b = 0;
  double time = .005;
 
  while(g < 255) {
    g++;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(r > 2) {
    r--;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(b < 255) {
    b++;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(g > 2) {
    g--;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(r < 255) {
    r++;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(b > 2) {
    b--;
    Vision5.setLedColor(r,g,b);
    wait(time,seconds);
  }
}

void brainRGB() {
  int hue = 0;
  int count = 0;
  int max = 325;
  Brain.Screen.setPenColor(black);
  
  while(count < max) {
    Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue++;
    count++;
  }
  while(count > 0) {
    Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue--;
    count--;
  }
}

void moveForward(int speed, double distanceToTravel) {
  double wheelDiameterIN = 3.25;
  double travelTargetCM = distanceToTravel; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumfrence) * sin(45);

  back_L.setVelocity(speed, vex::velocityUnits::pct);
  back_R.setVelocity(speed, vex::velocityUnits::pct);
  front_L.setVelocity(speed, vex::velocityUnits::pct);
  front_R.setVelocity(speed, vex::velocityUnits::pct);

  back_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  back_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  front_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}

void moveForwardSimple(int speed) {
  back_L.spin(fwd, speed, pct);
  back_R.spin(fwd, speed, pct);
  front_L.spin(fwd, speed, pct);
  front_R.spin(fwd, speed, pct);
}

void strafeSimpleRight(int speed) {
  back_R.spin(fwd, speed, pct);
  front_L.spin(fwd, speed, pct);
}

void strafeSimpleLeft(int speed) {
  back_L.spin(fwd, speed, pct);
  front_R.spin(fwd, speed, pct);
}

const double minimum_velocity = 15.0;

double increasing_speed(double starting_point, double current_position) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 50.0;
  return acceleration_constant * fabs(current_position - starting_point) +
    minimum_velocity;
}

double decreasing_speed(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 30.0;
  return deceleration_constant * fabs(current_position - ending_point) +
    minimum_velocity;
}

int angleConvertor(double ticks) {
  double ticksPerTurn = 1600; //2050
  double angleOfRobot = (ticks * 360) / ticksPerTurn;
  return angleOfRobot;
}

int conversion(double degree) {
  double ticksPerTurn = 1810; //2050
  double ticks = (degree * ticksPerTurn) / 360;
  double degreesToRotate = ticks;
  return degreesToRotate;

}

int get_average_encoder() {
  int position = ((((back_L.rotation(deg)) - (((back_R.rotation(deg))))))) / 2;
  return position;
}

float get_average_inertial() {
  float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
  //printf("heading average  %f\n", get_average_inertial());
  return robotDirection;
}

double headingError;
double headingErrorTest;

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply) {

  static
  const double circumference = 3.14159 * 3.25;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = (distanceIn / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double leftStartPoint = (front_L.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = back_L.rotation(rotationUnits::rev);
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = front_R.rotation(rotationUnits::rev);
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = back_R.rotation(rotationUnits::rev);
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  int sameEncoderValue = 0;

  front_R.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_R.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  front_L.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_L.spin(fwd, direction * minimum_velocity, velocityUnits::pct);

  task::sleep(120);
  printf("front right encoder %f\n", front_R.rotation(rev));
  double distanceTraveled = 0;

  while (direction * (distanceTraveled - rightStartPoint) < direction * wheelRevs) {

    if (back_R.velocity(rpm) == 0) {
      ++sameEncoderValue;
    }

    if (sameEncoderValue > 2) {
      break;
    }

    double rotationRight = pow(front_R.rotation(rev), 2);
    double rotationLeft = pow(front_L.rotation(rev), 2);
    distanceTraveled = direction * sqrt(rotationLeft + rotationRight);

    double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
    double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
    double error = (driftLeftError - driftRightError) / 2;

    if (error > -2.5 && error < 2.5) {
      headingErrorTest = direction * 0.5;
    } else {
      headingErrorTest = direction * 0.5;
    }

    headingError = (headingOfRobot - get_average_inertial()) * multiply;
    printf("heading error %f\n", error);
    printf("encoder value %f\n", distanceTraveled);
    printf("encoder error %f\n", rightEndPoint);

    if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      front_L.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled),
            decreasing_speed(leftEndPoint,
              distanceTraveled))) +
        (headingErrorTest + headingError),
        vex::velocityUnits::pct);
    } else {
      front_L.stop(hold);
    }

    if (direction *
      (distanceTraveled - rightStartPoint) <
      direction * wheelRevs) {
      front_R.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(
              rightStartPoint,
              distanceTraveled),
            decreasing_speed(rightEndPoint,
              distanceTraveled))) -
        (headingErrorTest + headingError),
        vex::velocityUnits::pct);
    } else {
      front_R.stop(hold);
    }

    if (direction *
      (distanceTraveled - leftStartPoint1) <
      direction * wheelRevs) {
      back_L.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(leftStartPoint1,
              distanceTraveled),
            decreasing_speed(leftEndPoint1,
              distanceTraveled))) -
        (headingErrorTest - headingError),
        vex::velocityUnits::pct);
    } else {
      back_L.stop(hold);
    }

    if (direction *
      (distanceTraveled - rightStartPoint1) <
      direction * wheelRevs) {
      back_R.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(rightStartPoint1,
              distanceTraveled),
            decreasing_speed(rightEndPoint1,
              distanceTraveled))) +
        (headingErrorTest - headingError),
        vex::velocityUnits::pct);
    } else {
      back_R.stop(hold);
    }
    task::sleep(10);
  }
  front_R.stop(hold);
  back_R.stop(hold);
  front_L.stop(hold);
  back_L.stop(hold);
}

void strafeWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply) {

  static
  const double circumference = 3.14159 * 3.25;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = (distanceIn / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double leftStartPoint = (front_L.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = back_L.rotation(rotationUnits::rev);
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = front_R.rotation(rotationUnits::rev);
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = back_R.rotation(rotationUnits::rev);
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  int sameEncoderValue = 0;

  front_R.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);
  front_L.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_R.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_L.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);

  task::sleep(120);
  printf("front right encoder %f\n", front_R.rotation(rev));
  printf("distance needed to travel %f\n", rightEndPoint);
  double distanceTraveled = 0;
  double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
  double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
  double previousOffset = (driftLeftError - driftRightError) / 2;

  while (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {

    if (back_R.velocity(rpm) == 0) {
      ++sameEncoderValue;
    }

    if (sameEncoderValue > 2) {
      //break;
    }

    double rotationLeft = pow(front_L.rotation(rev), 2);
    double rotationLeftBack = pow(back_L.rotation(rev), 2);
    distanceTraveled = direction * sqrt(rotationLeft + rotationLeftBack);

    double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
    double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
    double error = ((driftLeftError - driftRightError) / 2);

    if (error > -2.5 && error < 2.5) {
      headingErrorTest = direction * 0;
    } else {
      headingErrorTest = direction * 0;
    }

    headingError = (headingOfRobot - get_average_inertial()) * multiply;
    printf("heading error %f\n", headingError);
    printf("encoder value %f\n", distanceTraveled);
    printf("encoder error %f\n", rightEndPoint);

    if (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {
      front_L.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled),
            decreasing_speed(leftEndPoint,
              distanceTraveled))) +
        (headingError),
        vex::velocityUnits::pct);
    } else {
      front_L.stop(hold);
    }

    if (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {
      front_R.setVelocity(
        direction * -(std::min(
            maxVelocity,
            std::min(increasing_speed(
                rightStartPoint,
                distanceTraveled),
              decreasing_speed(rightEndPoint,
                distanceTraveled))) -
          (headingError)),
        vex::velocityUnits::pct);
    } else {
      front_R.stop(hold);
    }

    if (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {
      back_L.setVelocity(
        direction * -(std::min(
            maxVelocity,
            std::min(increasing_speed(leftStartPoint1,
                distanceTraveled),
              decreasing_speed(leftEndPoint1,
                distanceTraveled))) +
          (headingError)),
        vex::velocityUnits::pct);
    } else {
      back_L.stop(hold);
    }

    if (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {
      back_R.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(rightStartPoint1,
              distanceTraveled),
            decreasing_speed(rightEndPoint1,
              distanceTraveled))) -
        (headingError),
        vex::velocityUnits::pct);
    } else {
      back_R.stop(hold);
    }
    task::sleep(10);
  }
  front_R.stop(hold);
  back_R.stop(hold);
  front_L.stop(hold);
  back_L.stop(hold);
}

bool exit_function = false;

PID sMovePid;

int iMovePid(int target) {
  sMovePid.kP = 0.5;
  sMovePid.kI = 0;
  sMovePid.kD = 0;

  sMovePid.current = get_average_inertial();
  sMovePid.error = target - sMovePid.current;
  sMovePid.integral += sMovePid.error;
  sMovePid.derivative = sMovePid.error - sMovePid.lastError;
  sMovePid.lastError = sMovePid.error;

  return (((sMovePid.error) * (sMovePid.kP)) + ((sMovePid.derivative) * (sMovePid.kD)) + ((sMovePid.integral) * (sMovePid.kI)));
}

PID sRotatePid;

int iRotatePid(int target) {
  sRotatePid.kP = 2.3;
  sRotatePid.kI = 0;
  sRotatePid.kD = 4;

  sRotatePid.current = get_average_inertial();
  sRotatePid.error = target - sRotatePid.current;
  sRotatePid.integral += sRotatePid.error;
  sRotatePid.derivative = sRotatePid.error - sRotatePid.lastError;
  sRotatePid.lastError = sRotatePid.error;

  return (((sRotatePid.error) * (sRotatePid.kP)) + ((sRotatePid.derivative) * (sRotatePid.kD)) + ((sRotatePid.integral) * (sRotatePid.kI)));
}

void wait_until_drive_settled(int angle) {
  wait(10, msec);
  int maxPower = 10;
  int maxError = 10;
  int waiting;
  wait(10, msec);
  waiting++;
  while (1 == 1) {
    if (fabs(angle - get_average_inertial()) < maxError) {
      break;
    } else {
      int PIDPower = iRotatePid(angle);
      int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
      front_R.spin(fwd, -power, velocityUnits::pct);
      back_R.spin(fwd, -power, velocityUnits::pct);
      front_L.spin(fwd, power, velocityUnits::pct);
      back_L.spin(fwd, power, velocityUnits::pct);
      wait(10, msec);
    }
    wait(10, msec);
  }
  front_R.stop(hold);
  back_R.stop(hold);
  front_L.stop(hold);
  back_L.stop(hold);
}

void rotatePID(int angle) {
  int maxError = 20;
  int maxPower = 80;
  int timer = 0;
  int minVelocity = 1;
  exit_function = false;

  sRotatePid.integral = 0;

  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    front_R.spin(fwd, -power, velocityUnits::pct);
    back_R.spin(fwd, -power, velocityUnits::pct);
    front_L.spin(fwd, power, velocityUnits::pct);
    back_L.spin(fwd, power, velocityUnits::pct);
    wait_until_drive_settled(angle);
    wait(10, msec);
  }

  front_R.stop(hold);
  back_R.stop(hold);
  front_L.stop(hold);
  back_L.stop(hold);

}

void rotatePID(int angle, int maxPower) {
  int maxError = 0;
  int timer = 0;
  int minVelocity = 1;
  exit_function = false;
  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    printf("heading  %f\n", inertial_Up.rotation());
    printf("heading Left  %f\n", inertial_Down.rotation());
    printf("heading average  %f\n", get_average_inertial());
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    back_L.spin(fwd,  -power, velocityUnits::pct);
    front_R.spin(fwd, power, velocityUnits::pct);
    front_L.spin(fwd, -power, velocityUnits::pct);
    back_R.spin(fwd,  power, velocityUnits::pct);
    if (timer > 500 && fabs(back_L.velocity(pct)) < minVelocity) {
      exit_function = true;
    }
    wait(10, msec);
    timer += 10;
  }

  front_R.stop(hold);
  back_R.stop(hold);
  front_L.stop(hold);
  back_L.stop(hold);
}

bool linedUp = false;
/*-----------------------------------------------------------------------------*/
/** @brief      Turn Bot towards object */
/*-----------------------------------------------------------------------------*/

bool reached = false;
/*-----------------------------------------------------------------------------*/
/** @brief      Go toward set color */
/*-----------------------------------------------------------------------------*/
void goTo(int sigNumber, int velocity) {
  visionCamera.setBrightness(80);
  visionCamera.setSignature(SIG_1);

  while (!reached) {
    visionCamera.takeSnapshot(SIG_1);
    printf("Object Count %i\n", visionCamera.objectCount);
    printf("Object height %i\n", visionCamera.largestObject.height);
    if (visionCamera.largestObject.exists) {
      if (visionCamera.largestObject.height < 95 && visionCamera.largestObject.height > 2) {
        moveForwardSimple(velocity);
      } else {
        reached = true;
        front_L.stop(hold); // Stop the left motor.
        front_R.stop(hold);
        back_L.stop(hold); // Stop the left motor.
        back_R.stop(hold);
      }
    }
    task::sleep(10);
  }
}

void ObjectLooker(int sigNumber, int speed) {
  visionCamera.setBrightness(80);
  visionCamera.setSignature(SIG_1);
  int centerFOV = 158;
  int offsetX = 5;
  while (!linedUp) {
    visionCamera.takeSnapshot(SIG_1);
    printf("Object Count %i\n", visionCamera.objectCount);
    if (visionCamera.largestObject.exists) {
      if (visionCamera.largestObject.centerX > centerFOV + offsetX) {
        strafeSimpleRight(speed);
      } else if (visionCamera.largestObject.centerX < centerFOV - offsetX) {
        strafeSimpleLeft(speed);
      } else {
        linedUp = true;
        goTo(1, speed);
      }
    }
    task::sleep(10);
  }
  linedUp = false;
  reached = false;
}

void strafeWhileTurning(int speed, double distance){

while(get_average_inertial() < 176){
  back_L.spin(fwd, speed, pct);
  front_R.spin(fwd, speed * 4, pct);
  front_L.spin(fwd, -speed * 4, pct);
  task::sleep(10);
}
strafeWalk(-100, 100, 180, 0.5);

}