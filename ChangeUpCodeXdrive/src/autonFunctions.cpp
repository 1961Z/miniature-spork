#include "vex.h"

#include "visionSensorCode.h"

#include "autonFunctions.h"

#include "tracking.h"

using namespace vex;

Tracking tracking;

int intakeSpeedPCT = 100;

int ballFinal = 0;

bool checkingI = true;

bool checkingT = true;

bool goingDown = false;

void inertialCalibration(){
  inertial_Up.calibrate();
   while (inertial_Up.isCalibrating()) {
     wait(100, msec);
  }
  inertial_Down.calibrate();
  while (inertial_Down.isCalibrating()) {
   wait(100, msec);
  }
}

void resetFunction() {
  back_L.resetRotation();
  back_R.resetRotation();
  front_L.resetRotation();
  front_R.resetRotation();
  verticalTracker.resetRotation();
  horizontalTracker.resetRotation();
}

void setDriveSpeed(int leftSpeed, int rightSpeed){
  front_L.spin(fwd, leftSpeed, velocityUnits::pct);
  front_R.spin(fwd, rightSpeed, velocityUnits::pct);
  back_L.spin(fwd, leftSpeed, velocityUnits::pct);
  back_R.spin(fwd, rightSpeed, velocityUnits::pct);
}


void holdDrive(){
  front_L.stop(hold);
  front_R.stop(hold);
  back_L.stop(hold);
  back_R.stop(hold);
}

void brakeDrive(){
  front_L.stop(brake);
  front_R.stop(brake);
  back_L.stop(brake);
  back_R.stop(brake);
}

void coastDrive(){
  front_L.stop(coast);
  front_R.stop(coast);
  back_L.stop(coast);
  back_R.stop(coast);
}

void setIntakeSpeed(int speed){
  intake_L.spin(fwd, speed, pct);
  intake_R.spin(fwd, speed, pct);
}

void brakeIntake(){
  intake_R.stop(brake);
  intake_L.stop(brake);
}

void setConveyorSpeed(int speed){
  conveyor_L.spin(fwd, speed, pct);
  conveyor_R.spin(fwd, speed, pct);
}

void brakeConveyor(){
  conveyor_L.stop(brake);
  conveyor_R.stop(brake);
}

int bcount() {
  if(goingDown == false) {
    if(checkingI == true && LineTrackerIntake.reflectivity() > 5) {
      ballFinal++;
      checkingI = false;
    }
    if(LineTrackerIntake.reflectivity() < 5) {
      checkingI = true;
    }

    if(checkingT == true && LineTrackerTop.reflectivity() < 8) {
      ballFinal--;
      checkingT = false;
    }
    if(LineTrackerTop.reflectivity() > 8) {
      checkingT = true;
    }
  } 
  // Brain.Screen.printAt(1, 20, "balli count: %d", ballI);
  // Brain.Screen.printAt(1, 40, "ballT count: %d", ballT);
  // Brain.Screen.printAt(1, 40, "intake line: %d", LineTrackerIntake.reflectivity());
  // Brain.Screen.printAt(1, 60, "middle line: %d", LineTrackerMiddle.reflectivity());
  // Brain.Screen.printAt(1, 80, "top line: %d", LineTrackerTop.reflectivity());
  Brain.Screen.printAt(1, 20, "ball count: %ld\n", ballFinal);

  return ballFinal;
}

void visionRGB() {
  double r = 0;
  double g = 0;
  double b = 0;
  double time = .005;

  while(g < 255) {
    g++;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(r > 2) {
    r--;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(b < 255) {
    b++;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(g > 2) {
    g--;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(r < 255) {
    r++;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
  while(b > 2) {
    b--;
    visionCamera.setLedColor(r,g,b);
    wait(time,seconds);
  }
}

void brainRGB() {
  int hue = 0;
  int count = 0;
  int max = 325;
  Brain.Screen.setPenColor(black);

  while(count < max) {
    // Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    // Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue++;
    count++;
  }
  while(count > 0) {
    // Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    // Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue--;
    count--;
  }
}

void deaccel(int speed, double dist, double strength){
  static
  const double circumference = 3.14159 * 2.75;
  if (dist == 0)
    return;
  double wheelRevs = ((dist) / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double startPoint = (verticalTracker.rotation(rotationUnits::rev)) * -1;
  double endPoint = startPoint + wheelRevs;

  double calcVel = speed;
  double subtractor = 0;
  int count = 0;

  while(calcVel > 1){
    subtractor = pow(strength, count); 
    calcVel -= subtractor;
    if(subtractor >= calcVel){
      calcVel = round(calcVel);
      while(calcVel > 0){
        calcVel--;
        count++;
      }
      break;
    }
    count++;
  }

  front_R.spin(fwd, speed, velocityUnits::pct);
  front_L.spin(fwd, speed, velocityUnits::pct);
  back_R.spin(fwd, speed, velocityUnits::pct);
  back_L.spin(fwd, speed, velocityUnits::pct);

  task::sleep(70);
  // printf("encoder %f\n", verticalTracker.position(rotationUnits::rev));
  bool run = true;
  calcVel = speed;
  subtractor = 0;

  while(run == true){
    printf("encoder %f\n", verticalTracker.rotation(rev));
    Brain.Screen.printAt(1, 40, "count %ld\n", count);
    if((endPoint - verticalTracker.position(rotationUnits::rev)) == count){
      resetFunction();      
      while(calcVel > 1){
        subtractor = pow(strength, verticalTracker.position(rotationUnits::rev)); 
        calcVel -= subtractor;
        front_R.spin(fwd, calcVel, velocityUnits::pct);
        front_L.spin(fwd, calcVel, velocityUnits::pct);
        back_R.spin(fwd, calcVel, velocityUnits::pct);
        back_L.spin(fwd, calcVel, velocityUnits::pct);
        if(subtractor >= calcVel){
          calcVel = round(calcVel);
          while(calcVel > 0){
            calcVel--;
            front_R.spin(fwd, calcVel, velocityUnits::pct);
            front_L.spin(fwd, calcVel, velocityUnits::pct);
            back_R.spin(fwd, calcVel, velocityUnits::pct);
            back_L.spin(fwd, calcVel, velocityUnits::pct);
          }
          break;
        }
      }
    }
  }
}

void moveForwardFast(int speed, double distanceToTravel) {
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

void moveForward(int speed, double distanceToTravel) {
  double wheelDiameterIN = 3.25;
  double travelTargetCM = distanceToTravel; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumfrence) * sin(45);

  back_L.setVelocity(speed, vex::velocityUnits::pct);
  back_R.setVelocity(speed, vex::velocityUnits::pct);
  front_L.setVelocity(speed, vex::velocityUnits::pct);
  front_R.setVelocity(speed, vex::velocityUnits::pct);

  back_L.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  back_R.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  front_L.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  front_R.rotateFor(-degreesToRotate, vex::rotationUnits::deg, true);
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

double increasing_speed(double starting_point, double current_position, double addingFactor) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 80.0;
  return acceleration_constant * fabs(current_position - starting_point) +
    (minimum_velocity + addingFactor);
}

double decreasing_speed(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 40.0;
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

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double multiplyForHorizontal, double addingFactor) {

  static
  const double circumference = 3.14159 * 2.85;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn) / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double leftStartPoint = (verticalTracker.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = (verticalTracker.rotation(rotationUnits::rev));
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = (verticalTracker.rotation(rotationUnits::rev));
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = (verticalTracker.rotation(rotationUnits::rev));
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  front_L.spin(fwd, direction * (minimum_velocity + 1), velocityUnits::pct);
  front_R.spin(fwd, direction * (minimum_velocity - 1), velocityUnits::pct);
  back_R.spin(fwd, direction * (minimum_velocity + 1), velocityUnits::pct);
  back_L.spin(fwd, direction * (minimum_velocity - 1), velocityUnits::pct);
  
  task::sleep(90);


  int sameEncoderValue = 0;
  //printf("front right encoder %f\n", front_R.rotation(deg));
  double distanceTraveled = 0;

  while (direction * (distanceTraveled - rightStartPoint) < direction * wheelRevs) {
    if (back_R.velocity(rpm) == 0 || back_L.velocity(rpm) == 0) {
      ++sameEncoderValue;
    }

    if (sameEncoderValue > 2) {
      break;
    }

    double rotationRight = pow(front_R.rotation(rev), 2);
    double rotationLeft = pow(front_L.rotation(rev), 2);
    distanceTraveled = -(verticalTracker.rotation(rev));

    double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
    double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
    double error = horizontalTracker.rotation(degrees);

    if (error > -2.5 && error < 2.5) {
      headingErrorTest = -error * multiplyForHorizontal;
    } else {
      headingErrorTest = -error * multiplyForHorizontal;
    }

    headingError = -(headingOfRobot - get_average_inertial()) * multiply;
    /*printf("heading error %f\n", std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(leftEndPoint,
              distanceTraveled))));*/
    //printf("heading multiplied error %f\n", headingErrorTest);
    //printf("encoder value %f\n", distanceTraveled);
    //printf("wheelRevs %f\n", wheelRevs);
    //printf("encoder error %f\n", rightEndPoint);

    printf("front Left %f\n", front_L.velocity(pct));
    printf("front Right %f\n", front_R.velocity(pct));
    printf("back Left %f\n", back_L.velocity(pct));
    printf("back Right %f\n", back_R.velocity(pct));


    if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      front_L.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(leftEndPoint,
              distanceTraveled))) +
        (headingError) - (headingErrorTest),
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
              distanceTraveled, addingFactor),
            decreasing_speed(rightEndPoint,
              distanceTraveled))) -
        (headingError) - (headingErrorTest),
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
              distanceTraveled, addingFactor),
            decreasing_speed(leftEndPoint1,
              distanceTraveled))) +
        (headingError) - (headingErrorTest),
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
              distanceTraveled, addingFactor),
            decreasing_speed(rightEndPoint1,
              distanceTraveled))) -
        (headingError) - (headingErrorTest),
        vex::velocityUnits::pct);
    } else {
      back_R.stop(hold);
    }
    task::sleep(10);
  }
  back_R.stop(hold);
  back_L.stop(hold);
  front_R.stop(hold);
  front_L.stop(hold);
  //rotatePID(headingOfRobot, 60);
}

void strafeWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double addingFactor) {

  static
  const double circumference = 3.14159 * 2.75;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn - (direction * 1)) / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double leftStartPoint = (horizontalTracker.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = (horizontalTracker.rotation(rotationUnits::rev));
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = (horizontalTracker.rotation(rotationUnits::rev));
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = (horizontalTracker.rotation(rotationUnits::rev));
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  int sameEncoderValue = 0;

  front_R.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);
  front_L.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_R.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_L.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);

  task::sleep(70);
  printf("front right encoder %f\n", front_R.rotation(rev));
  printf("distance needed to travel %f\n", rightEndPoint);
  double distanceTraveled = 0;
  //double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
  //double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
  //double previousOffset = (driftLeftError - driftRightError) / 2;

  while (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {

    if (back_R.velocity(rpm) == 0) {
      ++sameEncoderValue;
    }

    if (sameEncoderValue > 2) {
      //break;
    }

    double rotationLeft = pow(front_L.rotation(rev), 2);
    double rotationLeftBack = pow(back_L.rotation(rev), 2);
    distanceTraveled = (horizontalTracker.rotation(rotationUnits::rev));

    double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
    double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
    double error = (verticalTracker.rotation(rotationUnits::rev));

    if (error > -2.5 && error < 2.5) {
      headingErrorTest = direction * 0;
    } else {
      headingErrorTest = direction * 0;
    }

    headingError = -(headingOfRobot - get_average_inertial()) * multiply;
    printf("heading error %f\n", headingError);
    printf("encoder value %f\n", distanceTraveled);
    printf("wheelRevs %f\n", wheelRevs);
    printf("encoder error %f\n", rightEndPoint);

    if (direction * (rightStartPoint + distanceTraveled) < direction * rightEndPoint) {
      front_L.setVelocity(
        direction * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled, addingFactor),
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
                distanceTraveled, addingFactor),
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
                distanceTraveled, addingFactor),
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
              distanceTraveled, addingFactor),
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
  //rotatePID(headingOfRobot, 60);
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
  //int waiting;
  wait(10, msec);
  //waiting++;
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
  visionCamera.setBrightness(50);
  visionCamera.setSignature(SIG_1);

  while (!reached) {
    visionCamera.takeSnapshot(SIG_1);
    printf("Object Count %ld\n", visionCamera.objectCount);
    printf("Object height %i\n", visionCamera.largestObject.height);
    if (visionCamera.largestObject.exists) {
      if (visionCamera.largestObject.height < 105 && visionCamera.largestObject.height > 2) {
        moveForwardSimple(velocity);
        if(back_L.velocity(pct) < 1){
          reached = true;
        }
      } 
      else {
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
  visionCamera.setBrightness(50);
  visionCamera.setSignature(SIG_1);
  int centerFOV = 158;
  int offsetX = 5;
  while (!linedUp) {
    visionCamera.takeSnapshot(SIG_1);
    printf("Object Count %ld\n", visionCamera.objectCount);
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
 while(get_average_inertial() < 89){ 
  back_L.spin(fwd, speed, pct); 
  front_R.spin(fwd, speed * 4, pct); 
  front_L.spin(fwd, -speed * 4, pct); 
  printf("heading average  %f\n", get_average_inertial()); 
  task::sleep(10); 
} 
strafeWalk(-10, 80, 90, 0.6, 0); 
}

int intakeOn() {
  while(true){
    intake_R.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    intake_L.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    if(LineTrackerIntake.reflectivity() >= 10) {
      task intakingBalls = task(scoreGoal);
    }
  }
}

void intakeOff(){
  intake_R.stop(brake);
  intake_L.stop(brake);
}

int intakeToggle() {
  while (true) {
    
    if (Controller1.ButtonR1.pressing()) {
      intake_R.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
      intake_L.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
      if(LineTrackerIntake.reflectivity() >= 4){
        task intakingBalls = task(scoreGoal);
        if(whenIntakingPrime == true) {
          task intakeAndScore = task(primeShoot);
        }
      }
    } 
    else if (Controller1.ButtonR2.pressing()) {
      intake_R.spin(directionType::rev, intakeSpeedPCT, voltageUnits::volt);
      intake_L.spin(directionType::rev, intakeSpeedPCT, voltageUnits::volt);
    } 
    else {
      intake_R.stop(brake);
      intake_L.stop(brake);
    }

    task::sleep(10);
  }
}

int timeKeeper;

void intakeMoves(){
 conveyor_L.rotateFor(fwd, 1, sec, 100, velocityUnits::pct);
 conveyor_R.rotateFor(fwd, 1, sec, 100, velocityUnits::pct);
}

bool waitTillOver = true; 
int threshold = 40;
bool cancel = false;

int primeShoot() {
  int timerBased = 0;
  cancel = false;
  while (true) {

    if (LineTrackerTop.reflectivity() < 10) {
      conveyor_L.spin(directionType::fwd, 100, velocityUnits::pct);
      conveyor_R.spin(directionType::fwd, 100, velocityUnits::pct);
    } 
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      waitTillOver = true; 
      break;
    }

    task::sleep(0);
    timerBased += 10;
  }
  return 1;
}

int goBackDown(){
 int timerCountDown = 0; 
  while(true){
     while (timerCountDown < 1000) {
      task::stop(intakeToggle);
      conveyor_L.spin(directionType::rev, 100, velocityUnits::pct);
      conveyor_R.spin(directionType::rev, 100, velocityUnits::pct);
      intake_L.spin(directionType::fwd, 100, velocityUnits::pct);
      intake_R.spin(directionType::fwd, 100, velocityUnits::pct);
      task::sleep(10);
      timerCountDown += 10;
      goingDown = true;
    }
    conveyor_L.stop(brake);
    conveyor_R.stop(brake);
    task::resume(intakeToggle);
    goingDown = false;
}
    
}

bool canceled = false; 

int scoreGoal(){
  int timerBased = 0;
  canceled = false;
  while (true) {
    // 79 middle 11 intake
    if (LineTrackerIntake.reflectivity() > 4 || (LineTrackerMiddle.reflectivity() < 9 && LineTrackerIntake.reflectivity() > 2)) {
      conveyor_L.spin(directionType::fwd, 100, velocityUnits::pct);
      conveyor_R.spin(directionType::fwd, 100, velocityUnits::pct);
    } 
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }

    task::sleep(0);
    timerBased += 10;
  }
  return 1;
}

bool whenIntakingPrime = false; 
bool startConveyorToGoDown = false;
int counterForSigs = 0; 


void primeShooterWithVision(){
  //visionCamera.setSignature(SIG_1);
  visionCamera.takeSnapshot(SIG_1);
  printf("Object Count %ld\n", visionCamera.objectCount);
  printf("Object height %i\n", visionCamera.largestObject.height);
  if(waitTillOver == false){ 
    if (visionCamera.largestObject.exists) {
      if (visionCamera.largestObject.height < 210 && visionCamera.largestObject.height > 120 && visionCamera.objectCount == 1) {
      task L = task(primeShoot);
      whenIntakingPrime = true;  
      startConveyorToGoDown = true; 

      } 
    }
  } else if(visionCamera.largestObject.height < 80 || !visionCamera.largestObject.exists) {
    whenIntakingPrime = false;
    int timerCountDown = 0;
    while (timerCountDown < 1000) {
      task::sleep(10);
      timerCountDown += 10;
    }
    timerCountDown = 0;
    while (timerCountDown < 1000 && startConveyorToGoDown == true) {
      task::stop(intakeToggle);
      conveyor_L.spin(directionType::rev, 100, velocityUnits::pct);
      conveyor_R.spin(directionType::rev, 100, velocityUnits::pct);
      intake_L.spin(directionType::fwd, 100, velocityUnits::pct);
      intake_R.spin(directionType::fwd, 100, velocityUnits::pct);
      task::sleep(10);
      timerCountDown += 10;   
    }
    conveyor_L.stop(brake);
    conveyor_R.stop(brake);
    intake_R.stop(brake);
    intake_L.stop(brake);
    task::resume(intakeToggle);
    startConveyorToGoDown = false; 
    whenIntakingPrime = false; 
    waitTillOver = false;
  }
}

void primShooterWithLimit() {
  if (goalChecker.pressing() && !(Controller1.ButtonA.pressing() || Controller1.ButtonL2.pressing() || Controller1.ButtonB.pressing()) && whenIntakingPrime == false) { 
    task L = task(primeShoot);
    whenIntakingPrime = true;
    startConveyorToGoDown = true;
  }

  else if (!goalChecker.pressing() && startConveyorToGoDown == true) {
    int timerCountDown = 0;
    while (timerCountDown < 1000 && startConveyorToGoDown == true) {
      task::stop(intakeToggle);
      conveyor_L.spin(directionType::rev, 100, velocityUnits::pct);
      conveyor_R.spin(directionType::rev, 100, velocityUnits::pct);
      intake_L.spin(directionType::fwd, 100, velocityUnits::pct);
      intake_R.spin(directionType::fwd, 100, velocityUnits::pct);
      task::sleep(10);
      if(timerCountDown == 1) {
        ballFinal++;
      }
      timerCountDown += 10;
    }
    conveyor_L.stop(brake);
    conveyor_R.stop(brake);
    intake_R.stop(brake);
    intake_L.stop(brake);
    task::resume(intakeToggle);
    startConveyorToGoDown = false;
    whenIntakingPrime = false; 
    waitTillOver = false;
  }
  else{
    task::resume(intakeToggle);
  }
}

int outtake1Ball() {
  conveyor_R.resetRotation(); 
  while (true) { 
    if (conveyor_R.rotation(rev) < 1.5) {
      conveyor_L.spin(fwd, 100, velocityUnits::pct);
      conveyor_R.spin(fwd, 100, velocityUnits::pct);
    }
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(10);
  }
  return 1; 
}

void outtake1BallAuton() {
  conveyor_R.resetRotation(); 
  while (true) { 
    if (conveyor_R.rotation(rev) < 1.5) {
      conveyor_L.spin(fwd, 100, velocityUnits::pct);
      conveyor_R.spin(fwd, 100, velocityUnits::pct);
    }
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(10);
  }
}

int outtake2Ball() {
  conveyor_R.resetRotation(); 
  while (true) { 
    if (conveyor_R.rotation(rev) < 4) {
      conveyor_L.spin(fwd, 100, velocityUnits::pct);
      conveyor_R.spin(fwd, 100, velocityUnits::pct);
    } else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(1);
  }
  return 1; 
}

void outtake2BallAuton() {
  conveyor_R.resetRotation(); 
  while (true) { 
    if (conveyor_R.rotation(rev) < 4) {
      conveyor_L.spin(fwd, 100, velocityUnits::pct);
      conveyor_R.spin(fwd, 100, velocityUnits::pct);
    } else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(1);
  } 
}

int outtake3Ball() {
  conveyor_R.resetRotation(); 
  while (true) {
    if (conveyor_R.rotation(rev) < 6) {
     conveyor_L.spin(fwd, 100, velocityUnits::pct);
     conveyor_R.spin(fwd, 100, velocityUnits::pct);
    } 
    
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(1);
  }
  return 1; 
}

void outtake3BallAuton() {
  conveyor_R.resetRotation(); 
  while (true) {
    if (conveyor_R.rotation(rev) < 6) {
     conveyor_L.spin(fwd, 100, velocityUnits::pct);
     conveyor_R.spin(fwd, 100, velocityUnits::pct);
    } 
    
    else {
      conveyor_L.stop(brake);
      conveyor_R.stop(brake);
      break;
    }
    task::sleep(1);
  } 
}

int BallCount(){
  while(true){
    bcount();
  }
}

void createBallCountTask(){
  task y = task(BallCount);
}

void stopBallCountTask(){
  task::stop(BallCount);
}

void createPrimeTask(){
  task poop = task(primeShoot);
}

void stopPrimeTask(){
  task::stop(primeShoot);
}

void createIntakeOnTask(){
  task ughh = task(intakeOn);
}

void stopIntakeOn(){
  task::stop(intakeOn);
}

void homeRowAuton(){ 

  createBallCountTask();
  createIntakeOnTask();
  moveForwardWalk(16, 80, 0, 0.6, 50, 0); //double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double multiplyForHorizontal, double addingFactor
  rotatePID(30, 90); //turning pid  int angle, int maxPower 
  stopIntakeOn();
  brakeIntake();
  createPrimeTask();
  moveForwardWalk(5, 80, 30, 0.6, 50, 0);
  stopPrimeTask();
  outtake1BallAuton();
  moveForward(70, 38);
  strafeWhileTurning(20, 24);
  moveForwardWalk(3, 80, 90, 0.6, 50, 0);
  outtake1BallAuton();
  moveForward(100, 20);
  strafeWalk(-30, 80, 90, 0.6, 0);
  rotatePID(135, 90);
  createIntakeOnTask();
  moveForwardFast(80,34);
  setConveyorSpeed(100);

}