#include "vex.h"

#include "visionSensorCode.h"

#include "autonFunctions.h"

#include "tracking.h"

using namespace vex;

Tracking tracking;

int intakeSpeedPCT = 100;

int ballFinal = 1;

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
  middle_L.resetRotation();
  middle_R.resetRotation();
  front_L.resetRotation();
  front_R.resetRotation();
  leftTracker.resetRotation();
  rightTracker.resetRotation();
}

int rainbowlogo() {

  while (1 == 1) {
    Brain.Screen.drawImageFromFile("BlueGroup1.png", 100, 0); // each picture is 270 by 258
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup2.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup3.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup4.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup5.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup6.png", 100, 0);
    task::sleep(1000);
  }
  task::sleep(1000);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Auton Selector */
/*-----------------------------------------------------------------------------*/

int autonomousSelection = -1;

typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;

// Button array definitions for each software button. The purpose of each button
// data structure is defined above.  The array size can be extended, so you can
// have as many buttons as you wish as long as it fits.

button buttons[] = {{30, 30, 60, 60, false, 0xE00000, 0x0000E0, "Ally"},
                    //{150, 30, 60, 60, false, 0x303030, 0xD0D0D0, ""},
                    //{270, 30, 60, 60, false, 0x303030, 0xF700FF, ""},
                    {390, 30, 60, 60, false, 0x303030, 0xDDDD00, "Back"},
                    {30, 150, 60, 60, false, 0x404040, 0xC0C0C0, "ReRun"},
                    // {150, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    //{270, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    {390, 150, 60, 60, false, 0x404040, 0xC0C0C0, "Skill"}};

// forward ref
void displayButtonControls(int index, bool pressed);

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button */
/*-----------------------------------------------------------------------------*/
int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;

    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states */
/*-----------------------------------------------------------------------------*/
void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackPressed() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    displayButtonControls(index, true);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackReleased() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    // clear all buttons to false, ie. unselected
    //      initButtons();

    // now set this one as true
    if (buttons[index].state == true) {
      buttons[index].state = false;
    } else {
      buttons[index].state = true;
    }

    // save as auton selection
    autonomousSelection = index;

    displayButtonControls(index, false);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons */
/*-----------------------------------------------------------------------------*/
void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                               buttons[i].width, buttons[i].height,
                               vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
                           buttons[i].ypos + buttons[i].height - 8,
                           buttons[i].label);
  }
}

void setDriveSpeed(int leftSpeed, int rightSpeed){
  leftDrive.spin(fwd, leftSpeed, velocityUnits::pct);
  rightDrive.spin(fwd, rightSpeed, velocityUnits::pct);
}

int debugging(){
  while(true){
    printf("frontL %f\n", front_L.velocity(pct));
    printf("frontR %f\n", front_R.velocity(pct));
    printf("backL %f\n", back_L.velocity(pct));
    printf("backR %f\n", back_R.velocity(pct));
    task::sleep(100);
  }
  task::sleep(10);
}

void holdDrive(){
  leftDrive.stop(hold);
  rightDrive.stop(hold);
}

void brakeDrive(){
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

void coastDrive(){
  leftDrive.stop(coast);
  rightDrive.stop(coast);
}

void setIntakeSpeed(int speed){
  intake.spin(fwd, speed, pct);
}

void brakeIntake(){
  intake.stop(brake);
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

    if(checkingT == true && LineTrackerIntake.reflectivity() < 8) {
      ballFinal--;
      checkingT = false;
    }
    if(LineTrackerIntake.reflectivity() > 8) {
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
  double startPoint = (leftTracker.rotation(rotationUnits::rev)) * -1;
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
  // printf("encoder %f\n", leftTracker.position(rotationUnits::rev));
  bool run = true;
  calcVel = speed;
  subtractor = 0;

  while(run == true){
    printf("encoder %f\n", leftTracker.rotation(rev));
    Brain.Screen.printAt(1, 40, "count %ld\n", count);
    if((endPoint - leftTracker.position(rotationUnits::rev)) == count){
      resetFunction();      
      while(calcVel > 1){
        subtractor = pow(strength, leftTracker.position(rotationUnits::rev)); 
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
  double degreesToRotate = ((360 * travelTargetCM) / circumfrence);

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

const double minimum_velocity = 5.0;

double increasing_speed(double starting_point, double current_position, double addingFactor) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 60.0;  //tuned 80
  return acceleration_constant * fabs(current_position - starting_point) +
    (5);
}

double increasing_speed_slow(double starting_point, double current_position, double addingFactor) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 60.0;  //tuned 80
  return acceleration_constant * fabs(current_position - starting_point) +
    (15);
}

double decreasing_speed(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 40.0; //tuned 30
  return deceleration_constant * fabs(current_position - ending_point) + 5;
}

double decreasing_speed_slow(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 35.0; //tuned 25
  return deceleration_constant * fabs(current_position - ending_point) + 5;
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

float get_average_encoder() {
  float position = ((leftTracker.rotation(rev)) + (rightTracker.rotation(rev))) / 2;
  return position;
}

float get_average_inertial() {
  float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
  //printf("heading average  %f\n", get_average_inertial());
  return robotDirection;
}

double calculateLeftSpeed(double speed, int turningRadius){
double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
double leftSpeed = (angularSpeed) * (turningRadius - 11.5);
leftSpeed = leftSpeed / 2;
return leftSpeed;
}

double calculateRightSpeed(double speed, int turningRadius){
double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
double rightSpeed = (angularSpeed) * (turningRadius + 11.5); 
rightSpeed = rightSpeed / 2; 
return rightSpeed;
}

PID sSpeedPid;

float iSpeedPid(float target, bool leftSide) {
  sSpeedPid.kP = 0.2; //2
  sSpeedPid.kI = 0;
  sSpeedPid.kD = 0;

  if(leftSide == true){
  sSpeedPid.current = leftDrive.velocity(pct);
  }
  else{
  sSpeedPid.current = rightDrive.velocity(pct);
  }
  sSpeedPid.error = target - sSpeedPid.current;
  if(leftSide == true){
  printf("error left %f\n", sSpeedPid.error);
  }
  else{
  printf("error right %f\n", sSpeedPid.error);
  }
  sSpeedPid.integral += sSpeedPid.error;
  sSpeedPid.derivative = sSpeedPid.error - sSpeedPid.lastError;
  sSpeedPid.lastError = sSpeedPid.error;

  return (((sSpeedPid.error) * (sSpeedPid.kP)) + ((sSpeedPid.derivative) * (sSpeedPid.kD)) + ((sSpeedPid.integral) * (sSpeedPid.kI)));
}

bool switchStatement = false; 

double headingError = 0;
double headingErrorTest = 0;
double pogChamp = 0;  
double rightTrackerError = 0;
double distanceTraveledlast = 0; 
double driftLeftError = 0, driftRightError = 0, combinedDriftError = 0, combinedDriftErrorMultiply = 0;  

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double multiplyForHorizontal = 0, double addingFactor = 0, bool cancel = true, int sideWays = 4, double turningRadius = 0, double angleNeeded = 0, double sideWaysDistance = 0, double stafeAtEnd = 0, double distanceAtEnd = 100, double angleAtEnd = 0, double turningRadiusAtEnd = 0) {

  static
  const double circumference = 3.14159 * 3.3 * 0.6;
  if (distanceIn == 0)
    return;
  double directionLeft = distanceIn > 0 ? 1.0 : -1.0;
  double directionRight = distanceIn > 0 ? 1.0 : -1.0;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn) / circumference);
  distanceAtEnd = distanceAtEnd / circumference; 
  resetFunction();
  double leftStartPoint = (leftTracker.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double rightStartPoint = (rightTracker.rotation(rotationUnits::rev));
  double rightEndPoint = rightStartPoint + wheelRevs;
  double leftSpeed, rightSpeed; 
  double offset = 0;

 switch(sideWays){
 case 2:
 break;
 case 3:  
 break; 
 case 4:
 break;
 }

  if(sideWays == 2){ 
  leftDrive.spin(fwd, directionLeft * calculateLeftSpeed(minimum_velocity, turningRadius), velocityUnits::pct);
  rightDrive.spin(fwd, directionRight *  calculateRightSpeed(minimum_velocity, turningRadius), velocityUnits::pct);
  printf("FrontL speed %f\n", calculateLeftSpeed(minimum_velocity, turningRadius));
  }
  else if(sideWays == 3){ 
  leftDrive.spin(fwd, directionLeft * calculateRightSpeed(minimum_velocity, turningRadius), velocityUnits::pct);
  rightDrive.spin(fwd, directionRight *  calculateLeftSpeed(minimum_velocity, turningRadius), velocityUnits::pct);
  }
  else{ 
  leftDrive.spin(fwd, directionLeft * (((minimum_velocity) * 1) * 1), pct);
  rightDrive.spin(fwd, directionRight *  (((minimum_velocity - offset) * 1) * 1), pct);
  }
  
  task::sleep(90);

  int sameEncoderValue = 0;
  double distanceTraveled = 0;

  while ((direction * (get_average_encoder() - rightStartPoint) <
       direction * wheelRevs) ||
      (direction * (get_average_encoder() - leftStartPoint) <
       direction * wheelRevs)) {
    if ((rightDrive.velocity(rpm) == 0 || leftDrive.velocity(rpm) == 0)) {
      ++sameEncoderValue;
    }

    if(direction == 1){
    if (sameEncoderValue > 10 ) {
      break;
    }
    }
    else{ 
       if (sameEncoderValue > 10 ) {
      break;
    }
    }

   /* if ((goalChecker.pressing()) && distanceTraveled > 0.1 && cancel == true) {
      break;
    }*/

    distanceTraveledlast = distanceTraveled; 
    distanceTraveled = (get_average_encoder());

    /*if(distanceTraveled > distanceAtEnd && switchStatement == false && fabs(get_average_inertial()) < angleAtEnd){
      sideWays = 2;
      turningRadius = turningRadiusAtEnd;
      switchStatement = true; 
    }

    else if(distanceTraveled > distanceAtEnd && switchStatement == false && fabs(get_average_inertial()) > angleAtEnd){
      printf("pog");
      sideWays = 3;
      turningRadius = turningRadiusAtEnd;
      switchStatement = true; 
    }
    
    if(sideWays >= 2 && sideWays < 4 && fabs(get_average_inertial()) > (angleNeeded) && switchStatement == false){    
      sideWays = 4; 
      headingOfRobot = angleNeeded; 
    }

    else if( fabs(get_average_inertial()) > (angleAtEnd) && switchStatement == true && sideWays == 2){
      printf("what\n");
      //switchStatement = false; 
      sideWays = 4; 
      headingOfRobot = angleAtEnd; 
    }

    else if( fabs(get_average_inertial()) < (angleAtEnd) && switchStatement == true && sideWays == 3){
      printf("what\n");
      //switchStatement = false; 
      sideWays = 4; 
      headingOfRobot = angleAtEnd;
    }*/

    if(((direction * wheelRevs) - 1.06) > distanceTraveled){
    headingError = -(headingOfRobot - get_average_inertial()) * multiply;
    }
    else{
    headingError = -(headingOfRobot - get_average_inertial()) * multiply;  
    }

    printf("left %f\n", leftDrive.velocity(pct));
    printf("right %f\n", rightDrive.velocity(pct));
  
    
  if(sideWays == 2){
    //printf("I get here\n");
    maxVelocity = 30;
    if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
     leftDrive.setVelocity(calculateLeftSpeed( 
        directionLeft * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(leftEndPoint,
              distanceTraveled))) +
        (headingError), turningRadius),
        vex::velocityUnits::pct);
    } else {
       holdDrive();
    }

    if (direction *
      (distanceTraveled - rightStartPoint) <
      direction * wheelRevs) {
      rightDrive.setVelocity(calculateRightSpeed(
        directionRight * std::min(
          maxVelocity,
          std::min(increasing_speed(rightStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(rightEndPoint,
              distanceTraveled))) -
        (headingError), turningRadius),
        vex::velocityUnits::pct);
    } else {
       holdDrive();
    }
    } 
    
    else if(sideWays == 3){
  
    maxVelocity = 30;
    if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      leftDrive.setVelocity(calculateRightSpeed( 
        directionLeft * std::min(
          maxVelocity,
          std::min(increasing_speed(
              leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(leftEndPoint,
              distanceTraveled))) +
        (headingError), turningRadius),
        vex::velocityUnits::pct);
    } else {
       holdDrive();
    }
    
    if (direction *
      (distanceTraveled - rightStartPoint) <
      direction * wheelRevs) {
      rightDrive.setVelocity(calculateLeftSpeed(
        directionRight * std::min(
          maxVelocity,
          std::min(increasing_speed(rightStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed(rightEndPoint,
              distanceTraveled))) -
        (headingError), turningRadius),
        vex::velocityUnits::pct);
    } else {
       holdDrive();
    }
    }
    else{
    if(direction == 1){
      //printf("i got here");
    if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      leftSpeed = (directionLeft * (std::min(maxVelocity,std::min(increasing_speed(leftStartPoint,distanceTraveled, addingFactor), decreasing_speed(leftEndPoint,distanceTraveled))) + (headingError))); 
      leftSpeed = ((leftSpeed * 1) * 1); 
      leftDrive.setVelocity(leftSpeed, pct); 
      if((direction * (wheelRevs - 0.2)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.2){ 
      leftSpeed += iSpeedPid(leftSpeed, true); }
      leftDrive.setVelocity(leftSpeed, pct); 
    } else {
      holdDrive();
    }
  
    if (direction *
      (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      rightSpeed = (directionLeft * (std::min(maxVelocity,std::min(increasing_speed(leftStartPoint,distanceTraveled, addingFactor), decreasing_speed(leftEndPoint,distanceTraveled))) - (headingError) - offset)); 
      rightSpeed = ((rightSpeed * 1) * 1);
      rightDrive.setVelocity(rightSpeed, pct);
      if((direction * (wheelRevs - 0.2)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.2){ 
      rightSpeed += iSpeedPid(rightSpeed, false); }
      rightDrive.setVelocity(rightSpeed, pct);
    } else {
      holdDrive();
    }
    } 
    else{
      if (direction * (distanceTraveled - leftStartPoint) <
      direction * wheelRevs) {
      leftDrive.setVelocity(
        directionLeft * std::min(
          maxVelocity,
          std::min(increasing_speed_slow(
              leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed_slow(leftEndPoint,
              distanceTraveled))) +
        (headingError),
        vex::velocityUnits::pct);
    } else {
      holdDrive();
    }
  
    if (direction *
      (distanceTraveled - rightStartPoint) <
      direction * wheelRevs) {
      rightDrive.setVelocity(
        directionRight * std::min(
          maxVelocity,
          std::min(increasing_speed_slow(leftStartPoint,
              distanceTraveled, addingFactor),
            decreasing_speed_slow(leftEndPoint,
              distanceTraveled))) -
        (headingError) - offset,
        vex::velocityUnits::pct);
    } else {
      holdDrive();
    }
    }
    }
    task::sleep(10);
  }
  holdDrive();
  switchStatement = false; 
}

/*
const double minimum_velocity = 15.0;

double
increasing_speed (double starting_point, double current_position,
		  double addingFactor)
{				// how fast the robot starts to pick up its speed
  static const double acceleration_constant = 600.0;
  return acceleration_constant * fabs (current_position - starting_point) +
    (minimum_velocity + addingFactor);
}

double
decreasing_speed (double ending_point, double current_position)
{				// how fast the robot starts to slow down before reaching its distance
  static const double deceleration_constant = 200.0;
  return deceleration_constant * fabs (current_position - ending_point) +
    minimum_velocity;
}

void
moveForwardWalk (double distanceIn, double maxVelocity)
{

  static const double circumference = 3.14159 * 2.85;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn) / circumference);
  //double left = 0 , right = 0 ; 
  //double lastEncoderValue = 0.0;\
  task::sleep (90);

  int sameEncoderValue = 0;
  double distanceTraveled = 0;

  double rightStartPoint = 0;

  double speedOfBot = 0;

  while (direction * (distanceTraveled - rightStartPoint) <
	 direction * wheelRevs)
    {

      distanceTraveled += 0.1;


      if (direction * (distanceTraveled - rightStartPoint) <
	  direction * wheelRevs)
	{
	  speedOfBot = direction * std::min (maxVelocity,
					     std::
					     min (increasing_speed
						  (rightStartPoint,
						   distanceTraveled, 0),
						  decreasing_speed (wheelRevs,
								    distanceTraveled)));
	}
      std::cout << "Speed of Bot is " << speedOfBot << std::endl;
      std::
	cout << " Distance Traveled is " << distanceTraveled << "\n" << std::
	endl;
    }
}





int
main ()
{
  moveForwardWalk (24, 80);

}
*/ // This is for testing on an online compiler to make sure the speeds are resonable 

void strafeWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double addingFactor) {

  static
  const double circumference = 3.14159 * 2.75;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn - (direction * 1)) / circumference);
  resetFunction();

  //double lastEncoderValue = 0.0;
  double leftStartPoint = (rightTracker.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = (rightTracker.rotation(rotationUnits::rev));
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = (rightTracker.rotation(rotationUnits::rev));
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = (rightTracker.rotation(rotationUnits::rev));
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  int sameEncoderValue = 0;

  front_L.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  back_L.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);
  front_R.spin(fwd, direction * -minimum_velocity, velocityUnits::pct);
  back_R.spin(fwd, direction * minimum_velocity, velocityUnits::pct);

  task::sleep(90);
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
    distanceTraveled = (rightTracker.rotation(rotationUnits::rev));

    double driftLeftError = (front_R.rotation(deg) + back_L.rotation(deg));
    double driftRightError = (front_L.rotation(deg) + back_R.rotation(deg));
    double error = (leftTracker.rotation(rotationUnits::rev));

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
  holdDrive();
  //rotatePID(headingOfRobot, 60);
}

void stafeThanForward(double speed, bool side){
  if(side == true){
  front_L.spin(fwd, speed, pct);
  back_R.spin(fwd, speed, pct);
  front_R.stop();
  back_L.stop();
  }
  else
  {
    setDriveSpeed(0, speed);
  }
}

void rightPivotTurn(int speed, int angle, double turningRadius){ 
double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
double leftSpeed = (angularSpeed) * (turningRadius - 11.5);
double rightSpeed = (angularSpeed) * (turningRadius + 11.5) ;  
 while(get_average_inertial() < angle){
  front_R.spin(fwd, rightSpeed, rpm);
  front_L.spin(fwd, leftSpeed, rpm);
  back_R.spin(fwd , rightSpeed, rpm);
  back_L.spin(fwd , leftSpeed, rpm);
  printf("left speed  %f\n", leftSpeed);
  printf("right speed  %f\n", rightSpeed);
  task::sleep(10);
 }
 holdDrive();
}

bool exit_function = false;



PID sRotatePid;

int iRotatePid(int target) {
  sRotatePid.kP = 0.51;
  sRotatePid.kI = 0;
  sRotatePid.kD = 0.32;

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
  double maxError = 0.1;
  int timer = 0;
  int minVelocity = 10;
  exit_function = false;
  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    printf("heading  %f\n", inertial_Up.rotation());
    printf("heading Left  %f\n", inertial_Down.rotation());
    printf("heading average  %f\n", get_average_inertial());
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    leftDrive.spin(fwd,  -power, velocityUnits::pct);
    rightDrive.spin(fwd, power, velocityUnits::pct);
    if (timer > 600 && fabs(leftDrive.velocity(pct)) < minVelocity) {
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
    intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    if(LineTrackerIntake.reflectivity() >= 10) {
      task intakingBalls = task(scoreGoal);
    }
  }
}

void intakeOff(){
  intake.stop(brake);
}

int whenToStop = 0;

int intakeToggle() {
  while (true) {

    if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      setIntakeSpeed(-100);
      indexer.spin(fwd, -100, pct);
      whenToStop = 1; 
    }
    else if (Controller1.ButtonR1.pressing()) {
      intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
      if(LineTrackerIntake.reflectivity() >= 17){
        task intakingBalls = task(scoreGoal);
      }
    } 
    else if (Controller1.ButtonR2.pressing()) {
      intake.spin(directionType::rev, intakeSpeedPCT, voltageUnits::volt);
    } 
    else {
      brakeIntake();
      if(whenToStop % 2 != 0){
      task g = task(outtake0Ball); 
      whenToStop = 0; 
      }
    }
    
    printf("leftfront: %i\n", whenToStop);
    task::sleep(10);
  }
}

int timeKeeper;

bool waitTillOver = false; 
int threshold = 40;
bool cancel = false;

// 79 middle 11 intake
    /*if(LineTracker.reflectivity() > 10){
      if(LineTrackerIntake.reflectivity() > 10){
        indexer.spin(fwd, 50, velocityUnits::pct);
      }
      else{
      indexer.stop(hold);
      break;
      }
    }
    else if (LineTrackerIntake.reflectivity() > 17 || ((LineTrackerMiddle.reflectivity() < 22 && LineTrackerIntake.reflectivity() > 2))) {
     
      indexer.spin(fwd, 50, velocityUnits::pct);

    } 

    else {
      indexer.stop(hold);
      break;
    }*/ 

bool canceled = false; 

int scoreGoal(){
  int timerBased = 0;
  canceled = false;
  while (true) {
    

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
      //task L = task(primeShoot);
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
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
      task::sleep(10);
      timerCountDown += 10;   
    }
    intake.stop(brake);
    task::resume(intakeToggle);
    startConveyorToGoDown = false; 
    whenIntakingPrime = false; 
    waitTillOver = false;
  }
}

int outtake0Ball() {
  indexer.resetRotation(); 
  while (true) { 
    if (fabs(indexer.rotation(rev)) < 0) {
      indexer.spin(fwd, 100, velocityUnits::pct);
    }
    else {
      indexer.stop(hold);
      break;
    }
    task::sleep(10);
  }
  return 1; 
}

int outtake1Ball() {
  indexer.resetRotation(); 
  while (true) { 
    if (fabs(indexer.rotation(rev)) < 2) {
      indexer.spin(fwd, 100, velocityUnits::pct);
      //conveyor_R.spin(fwd, 100, velocityUnits::pct);
    }
    else {
      indexer.stop(brake);
      break;
    }
    task::sleep(10);
  }
  return 1; 
}

void outtake1BallAuton() {
  indexer.resetRotation(); 
  while (true) { 
    if (indexer.rotation(rev) < 2) {
      indexer.spin(fwd, 100, velocityUnits::pct);
    } else {
       indexer.stop(brake);
      break;
    }
    task::sleep(10);
  } 
}


int outtake2Ball() {
  indexer.resetRotation(); 
  while (true) { 
    if (fabs(indexer.rotation(rev)) < 5.5) {
      indexer.spin(fwd, 100, velocityUnits::pct);
      //conveyor_R.spin(fwd, 100, velocityUnits::pct);
    } else {
      indexer.stop(brake);
      //conveyor_R.stop(brake);
      break;
    }
    task::sleep(1);
  }
  return 1; 
}

void outtake2BallAuton() {
  indexer.resetRotation(); 
  while (true) { 
    if (indexer.rotation(rev) < 5.5) {
      indexer.spin(fwd, 100, velocityUnits::pct);
    } else {
       indexer.stop(brake);
      break;
    }
    task::sleep(10);
  } 
}

int outtake3Ball() {
  indexerRight.resetRotation(); 
  while (true) {
    if (indexerRight.rotation(rev) < 6) {
     indexer.spin(fwd, 100, velocityUnits::pct);
    } 
    
    else {
      indexer.stop(brake);
      break;
    }
    task::sleep(1);
  }
  return 1; 
}

void outtake3BallAuton() {
  indexer.resetRotation(); 
  while (true) {
    if (indexerRight.rotation(rev) < 6) {
     indexer.spin(fwd, 100, velocityUnits::pct);
    } 
    
    else {
      indexer.stop(brake);
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

/*void createPrimeTask(){
  task poop = task(primeShoot);
}

void stopPrimeTask(){
  task::stop(primeShoot);
}*/

void createIntakeOnTask(){
  task ughh = task(intakeOn);
}

void stopIntakeOn(){

  task::stop(intakeOn);
  brakeIntake();

}

int stopIntakeFunction(){ 
  int x = 0; 
  int timeLimit = 1250;
  while(x < timeLimit){
    x+= 10 ; 
    task::sleep(10);
  }
  if(x >= timeLimit){
    stopIntakeOn();
  }
  return 1;
}

int stopIntakeFunction2nd(){ 
  int x = 0; 
  int timeLimit = 1000;
  while(x < timeLimit){
    x+= 10 ; 
    task::sleep(10);
  }
  if(x >= timeLimit){
    stopIntakeOn();
  }
  return 1;
}

void outtakeIntakes(double revolutions, int speed){ 
  intake.rotateFor(fwd, revolutions, rev, speed, velocityUnits::pct, false);
}

void preAuton() {

  inertial_Up.calibrate();
  while(inertial_Up.isCalibrating()){
    wait(100, msec);
  }
  inertial_Down.calibrate();
  while(inertial_Down.isCalibrating()){
    wait(100, msec);
  }

  Controller1.Screen.setCursor(1, 0);
  Controller1.Screen.print("IMU done calibrating");

  displayButtonControls(0, true);
  Brain.Screen.pressed(userTouchCallbackPressed);
  Brain.Screen.released(userTouchCallbackReleased);
  rainbowlogo();
  resetFunction();
}

void homeRowAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  task::sleep(200);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  moveForwardWalk(32, 80, 0, 0.6);
  rotatePID(90, 80);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(30, 80, 90, 0.6);
  outtake2BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  moveForwardWalk(-24, 80, 90, 0.6);
  brakeIntake();
  rotatePID(225, 80);
  moveForwardWalk(85, 80, 225, 0.6);
  rotatePID(180, 80);
  createIntakeOnTask();
  task st = task(stopIntakeFunction2nd);
  moveForwardWalk(28, 80, 180, 0.6);
  outtake2BallAuton();
  task::stop(st);
  intake.spin(reverse, 100, pct);
  moveForwardWalk(-24, 80, 180, 0.6);
  brakeIntake();

  
}


void skills(){
  createBallCountTask();
  indexer.spin(fwd, 100, pct);
  task::sleep(200);
  indexer.stop(); 
  createIntakeOnTask();
  moveForwardWalk(12, 80, 0, 0.6);
  rotatePID(45, 80);
  moveForwardWalk(24, 80, 45, 0.6);
  stopIntakeOn(); 
  brakeIntake(); 
  rotatePID(90, 80);
  moveForwardWalk(14, 80, 90, 0.6);
  outtake2BallAuton();
  moveForwardWalk(-16, 80, 90, 0.6);
  createIntakeOnTask();
  rotatePID(-45, 80);
  moveForwardWalk(46, 80, -45, 0.6);
  rotatePID(45, 80);
  stopIntakeOn();
  moveForwardWalk(7, 80, 45, 0.6);
  outtake2BallAuton();
  moveForwardWalk(-22, 80, 45, 0.6);
  rotatePID(-45, 80);
  createIntakeOnTask();
  moveForwardWalk(48, 80, -45, 0.6);
  moveForwardWalk(-14, 80, -45, 0.6);
  rotatePID(0, 80);
  stopIntakeOn();
  moveForwardWalk(26, 80, 0, 0.6);
  outtake2BallAuton();
  moveForwardWalk(-46, 80, 0, 0.6);
  rotatePID(-135, 80);
  createIntakeOnTask();
  moveForwardWalk(24, 80, -135, 0.6);
  rotatePID(-45, 80);
  stopIntakeOn();
  moveForwardWalk(26, 80, -45, 0.6);
  outtake2Ball();
  moveForwardWalk(-10, 80, -45, 0.6);
  rotatePID(-135, 80);
  createIntakeOnTask();
  moveForwardWalk(46, 80, -135, 0.6);
  rotatePID(-90, 80);
  stopIntakeOn();
  moveForwardWalk(12, 80, -90, 0.6);
  outtake2BallAuton();
  moveForwardWalk(-16, 80, -90, 0.6);
  rotatePID(-225, 80);
  createIntakeOnTask();
  moveForwardWalk(44, 80, -225, 0.6);
  rotatePID(-135, 80);
  stopIntakeOn();
  moveForwardWalk(5, 80, -135, 0.6);
  outtake2BallAuton();
  moveForwardWalk(-12, 80, -135, 0.6);
  createIntakeOnTask();
  rotatePID(45, 80);
  moveForwardWalk(10, 80, 45, 0.6);
  rotatePID(-150, 80);
  moveForwardWalk(-20, 80, -150, 0.6);
  moveForwardWalk(20, 80, -150, 0);
  rotatePID(-225, 80);
  moveForwardWalk(48, 80, -225, 0.6);
  moveForwardWalk(-12, 80, -225, 0.6);
  stopIntakeOn();
  rotatePID(-180, 80);
  moveForwardWalk(26, 80, -180, 0.6);
  outtake1BallAuton();
  moveForwardWalk(-48, 80, -180, 0.6);
  rotatePID(0, 80);
  moveForwardWalk(24, 80, 0, 0.6);
  outtake1BallAuton();
  moveForwardWalk(-24, 80, 0, 0.6);
  
}

void testAuton(){ 
  //moveForwardWalk(72, 90, 0, 0.6);
  //moveForward(80, 52);
  //task::sleep(500);
  //moveForward(40, -360);
  //task::sleep(500);
  rotatePID(90, 80);
  moveForwardWalk(72, 80, 90, 0.6);
  task::sleep(200);
  moveForwardWalk(-72, 80, 90, 0.6);
  task::sleep(200);
  rotatePID(0, 80);
}
