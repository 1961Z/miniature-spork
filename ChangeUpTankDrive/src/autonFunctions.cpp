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


int ballC = 0;

bool senseTop = false;
bool senseBottom = false;

int bcount() {
  while (true) {
      printf("ball count: %id\n", ballC);
      if (ballC <= 3 && ballC >= 0) {
        if (LineTrackerIntake.reflectivity() > 17 && senseBottom == false) {
          ballC++;
          senseBottom = true;
        } else if (LineTrackerIntake.reflectivity() < 10) {
          senseBottom = false;
        }

        /*if (LineTrackerOuttake.reflectivity() > 99 && senseTop == false) {
          ballC--;
          senseTop = true;
        } else if (LineTrackerOuttake.reflectivity() < 100) {
          senseTop = false;
        }*/
      }
      task::sleep(10);
  }
  return ballC;
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
  float position = (-(leftTracker.rotation(rev)) + (rightTracker.rotation(rev))) / 2;
  return position;
}

float get_average_encoder_deg() {
  float position = (-(leftTracker.rotation(degrees)) + (rightTracker.rotation(degrees))) / 2;
  return position;
}

float get_average_inertial() {
  float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
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

double increasing_speed(double starting_point, double current_position, double addingFactor) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 60.0;  //tuned 80
  return acceleration_constant * fabs(current_position - starting_point) +
    (50);
}

double increasing_speed_slow(double starting_point, double current_position, double addingFactor) { // how fast the robot starts to pick up its speed
  static
  const double acceleration_constant = 100.0;  //tuned 80
  return acceleration_constant * fabs(current_position - starting_point) +
    (90);
}

double decreasing_speed(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 31.0; //tuned 30
  return deceleration_constant * fabs(current_position - ending_point) + 5;
}

double decreasing_speed_slow(double ending_point, double current_position) { // how fast the robot starts to slow down before reaching its distance
  static
  const double deceleration_constant = 40.0; //tuned 25
  return deceleration_constant * fabs(current_position - ending_point) + 5;
}

PID sHeadingPid;

int iHeadingPid(float target) {
  sHeadingPid.kP = 4;
  sHeadingPid.kI = 0;
  sHeadingPid.kD = 1.6;

  sHeadingPid.current = get_average_inertial();
  sHeadingPid.error = target - sHeadingPid.current;
  sHeadingPid.integral += sHeadingPid.error;
  sHeadingPid.derivative = sHeadingPid.error - sHeadingPid.lastError;
  sHeadingPid.lastError = sHeadingPid.error;

  return (((sHeadingPid.error) * (sHeadingPid.kP)) + ((sHeadingPid.derivative) * (sHeadingPid.kD)) + ((sHeadingPid.integral) * (sHeadingPid.kI)));
}

PID sMovePid;

int iMovePid(float target) {
  sMovePid.kP = 0.13;
  sMovePid.kI = 0;
  sMovePid.kD = 0.2;

  sMovePid.current = get_average_encoder_deg();
  sMovePid.error = target - sMovePid.current;
  sMovePid.integral += sMovePid.error;
  sMovePid.derivative = sMovePid.error - sMovePid.lastError;
  sMovePid.lastError = sMovePid.error;

  return (((sMovePid.error) * (sMovePid.kP)) + ((sMovePid.derivative) * (sMovePid.kD)) + ((sMovePid.integral) * (sMovePid.kI)));
}

PID sSpeedPid;

float iSpeedPid(float target, bool leftSide) {
  sSpeedPid.kP = 0.16; //2
  sSpeedPid.kD = 0;
  sSpeedPid.kI = 0;

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

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, bool cancel = true, double multiplyForHorizontal = 0, double addingFactor = 0, int sideWays = 4, double turningRadius = 0, double angleNeeded = 0, double sideWaysDistance = 0, double stafeAtEnd = 0, double distanceAtEnd = 100, double angleAtEnd = 0, double turningRadiusAtEnd = 0) {

  static const double circumference = 3.14159 * 2.77;
  if (distanceIn == 0)
    return;
  double directionLeft = distanceIn > 0 ? 1.0 : -1.0;
  double directionRight = distanceIn > 0 ? 1.0 : -1.0;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = ((distanceIn) / circumference);
  double wheelRevsDegree = ((360 * distanceIn) / circumference);
  distanceAtEnd = distanceAtEnd / circumference; 
  resetFunction();
  double leftStartPoint = (leftTracker.rotation(rotationUnits::rev));
  double leftEndPoint = leftStartPoint + wheelRevs;
  double rightStartPoint = (rightTracker.rotation(rotationUnits::rev));
  double rightEndPoint = rightStartPoint + wheelRevs;
  double leftSpeed, rightSpeed; 
  double offset = 0;
  double realSpeed = 0; 

 switch(sideWays){
 case 2:
 break;
 case 3:  
 break; 
 case 4:
 break;
 }

  int sameEncoderValue = 0;
  double distanceTraveled = 0;
  double PIDPowerL, PIDPowerR, PIDPowerHeading; 

  while ((direction * (get_average_encoder() - rightStartPoint) < direction * wheelRevs) || (direction * (get_average_encoder() - leftStartPoint) < direction * wheelRevs)) {
    

   /* if ((goalChecker.pressing()) && distanceTraveled > 0.1 && cancel == true) {
      break;
    }*/

    distanceTraveled = (get_average_encoder());


    if ((goalChecker.reflectivity()  > 3 || (fabs(leftDrive.velocity(pct)) < 1) || (fabs(rightDrive.velocity(pct)) < 1)) && cancel == true && fabs(distanceTraveled) > 0.1) {
      ++sameEncoderValue;
    }

    if(sameEncoderValue > 10){
      break;
    }

    //y\ =\ 2\cos\left(\frac{x}{20}\right)\ +2

    PIDPowerHeading = iHeadingPid(headingOfRobot);
    headingError = fabs(PIDPowerHeading) < multiply ? PIDPowerHeading : multiply * (PIDPowerHeading / fabs(PIDPowerHeading));

    if (direction == fabs(direction)) {
      headingError = headingError;
    } else {
      headingError = headingError;
    }

    printf("heading %f\n", get_average_inertial());
    printf("headingError %f\n", PIDPowerHeading); 
    //printf("distanceTraveled %f\n", distanceTraveled);

      if (direction * (distanceTraveled - leftStartPoint) < direction * wheelRevs) {
        if ((fabs(distanceTraveled) < 0.8)) {
          if(direction == fabs(direction)){ 
          leftSpeed = maxVelocity; 
          }
          else{
          leftSpeed = std::min(increasing_speed(leftStartPoint, distanceTraveled, offset), maxVelocity);
          }
          leftSpeed = (directionLeft * (leftSpeed - headingError));
          front_L.spin(fwd, leftSpeed, pct);
          back_L.spin(fwd, leftSpeed, pct);
        } else {
          PIDPowerL = iMovePid(wheelRevsDegree);
          PIDPowerL = fabs(PIDPowerL) < maxVelocity ? PIDPowerL : maxVelocity * (PIDPowerL / fabs(PIDPowerL));
          printf("left Speed %f\n", PIDPowerL);
          PIDPowerL = ((PIDPowerL - headingError));
          front_L.spin(fwd, (PIDPowerL * 0.01) * 12, volt);
          back_L.spin(fwd, (PIDPowerL * 0.01) * 12, volt);
        }
        //printf("left Speed Real %f\n", leftDrive.velocity(pct));
        /*if ((direction * (wheelRevs - 0.1)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.1) {
          //leftSpeed += iSpeedPid(leftSpeed, true);
        }
        //leftDrive.spin(fwd, leftSpeed, pct);*/
      } else {
        brakeDrive();
      }
      if (direction * (distanceTraveled - rightStartPoint) < direction * wheelRevs) {
        if ((fabs(distanceTraveled) < 0.8)) {
          if(direction == fabs(direction)){
          rightSpeed = maxVelocity; 
          }
          else{
          rightSpeed = std::min(increasing_speed(leftStartPoint, distanceTraveled, offset), maxVelocity);
          }
          rightSpeed = (directionLeft * (rightSpeed + headingError));
          front_R.spin(fwd, rightSpeed, pct);
          back_R.spin(fwd, rightSpeed, pct);
        } else {
          PIDPowerR = iMovePid( wheelRevsDegree);
          PIDPowerR = fabs(PIDPowerR) < maxVelocity ? PIDPowerR : maxVelocity * (PIDPowerR / fabs(PIDPowerR));
          PIDPowerR = ((PIDPowerR + headingError));
          printf("right Speed %f\n", PIDPowerR);
          front_R.spin(fwd, (PIDPowerR * 0.01) * 12, volt);
          back_R.spin(fwd, (PIDPowerR * 0.01) * 12, volt);
        }
       // printf("right Speed Real %f\n", rightDrive.velocity(pct));
        /*if ((direction * (wheelRevs - 0.1)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.1) {
          //rightSpeed += iSpeedPid(rightSpeed, false);
        }
        //rightDrive.spin(fwd, rightSpeed, pct);*/
      } else {
        brakeDrive();
      }
      distanceTraveledlast = distanceTraveled; 
      task::sleep(10);
    }
  brakeDrive();
  switchStatement = false; 
}

bool exit_function = false;

PID sRotatePid;
double;

int iRotatePid(int target) {
  sRotatePid.kP = 0.49;
  sRotatePid.kI = 0;
  sRotatePid.kD = 0.;

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
  double minVelocity = 0.5;
  exit_function = false;
  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    printf("heading average  %f\n", get_average_inertial());
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    leftDrive.spin(fwd,  -power, velocityUnits::pct);
    rightDrive.spin(fwd, power, velocityUnits::pct);
    if (timer > 800 && fabs(leftDrive.velocity(pct)) < minVelocity) {
      exit_function = true;
    }
    wait(10, msec);
    timer += 10;
  }
  brakeDrive();
}

void rotatePIDWack(int angle, int maxPower, bool leftSide) {
  double maxError = 0.1;
  int timer = 0;
  double minVelocity = 4;
  exit_function = false;
  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    printf("heading average  %f\n", get_average_inertial());
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    if(leftSide == 1){
    rightDrive.stop(hold);
    leftDrive.spin(fwd,  -power, velocityUnits::pct);
    }
    else{
    rightDrive.spin(fwd, power, velocityUnits::pct);
    }
    if (timer > 600 && fabs(leftDrive.velocity(pct)) < minVelocity) {
      exit_function = true;
    }
    wait(10, msec);
    timer += 10;
  }
  brakeDrive();
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

bool whenIntakingPrime = false; 
bool startConveyorToGoDown = false;
int counterForSigs = 0; 

int timeKeeper;

bool waitTillOver = false; 
int threshold = 40;
bool cancel = false;

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
      task intakingBalls = task(scoreGoal);
  }
}

int whenToStop = 0;

int intakeToggle() {
  while (true) {
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing() && (indexer.velocity(pct) < 100)){
      setIntakeSpeed(-100);
      indexer.spin(fwd, -100, pct);
      ballC = 0;
      whenToStop = 1; 
      task::stop(scoreGoal);
    }
    else if (Controller1.ButtonR1.pressing()) {
      intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
      if(LineTrackerIntake.reflectivity() > 17){
      task intakingBalls = task(scoreGoal);
      }
    } 
    else if (Controller1.ButtonR2.pressing()) {
      intake.spin(directionType::rev, intakeSpeedPCT, voltageUnits::volt);
    }
    else if(Controller1.ButtonUp.pressing()){
      task intakingBalls = task(scoreGoal);
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

bool canceled = false;
bool tip = false;
bool booga = false;

int scoreGoal() {
  indexer.resetRotation();
  int x = 0;
  while (true) {
    if (goalChecker.reflectivity() < 99) {
      if (ballC == 1 || ballC == 2) {
        if (!(topConveyor.pressing())) {
          tip = true;
          indexer.spin(fwd, 100, pct);
        } else if (topConveyor.pressing() && tip == true) {
          indexerTop.spin(fwd, 100, pct);
          task::sleep(40);
          tip = false;
          indexerTop.stop(hold);
        } else if (indexerBottom.rotation(rev) < 2.2 && ballC == 2) {
          indexerTop.stop(hold);
          indexerBottom.spin(fwd, 100, pct);
        } else {
          indexer.stop(hold);
          break;
        }
      }
    } else {
      break;
    }
    task::sleep(10);
  }
  return 1;
}

int outtake0Ball() {
  indexer.resetRotation();
  while (true) {
    if (fabs(indexer.rotation(rev)) < 0) {
      indexer.spin(fwd, 100, velocityUnits::pct);
    } else {
      indexer.stop(hold);
      break;
    }
    task::sleep(10);
  }
  return 1;
}

void outtake1BallAuton() {
  indexer.resetRotation();
  while (true) {
      if (indexer.rotation(rev) < 3.2) {
        indexer.spin(fwd, 100, pct);
      } else {
        indexer.stop(brake);
        ballC--;
        break;
      }
    }
}

int outtake1Ball() {
  indexer.resetRotation();
  int x = 0;
  int ballCount = ballC;
  if(ballCount >= 1){
  indexerTop.spin(fwd, 100, pct);
  task::sleep(100);
  }
  while (true) {
      if( !(topConveyor.pressing()) && ballCount > 1){
       indexer.spin(fwd, 100, pct);
       printf("ooga");
      }
      else if (indexer.rotation(rev) < 2 && ballCount == 1) {
        indexer.spin(fwd, 100, pct);
      } 
      else {
        if(ballCount >= 1){
        indexer.spin(fwd, 100, pct);
        task::sleep(50);
        indexerTop.stop(hold);
        indexerBottom.stop(hold);
        }
        ballC--;  
        break;
      }
    task::sleep(10);
    x+=10; 
    }
  return 1;
}

int outtake2Ball() {
  indexerTop.resetRotation();
  int x = 0;
  int ballCount = ballC;
  while (true) {
    if (fabs(indexerTop.rotation(rev)) < 9) {
      if (ballCount == 2) {
        indexer.spin(fwd, 100, pct);
      } else {
        indexerTop.spin(fwd, 100, velocityUnits::pct);
        if (!topConveyor.pressing()) {
          while (x < 200) {
            task::sleep(100);
            x += 100;
          }
          indexerBottom.spin(fwd, 100, pct);
        } else {
          indexerBottom.stop(brake);
        }
      }
    } else {
      indexer.stop(brake);
      ballC -= 2;
      break;
    }
    task::sleep(1);
  }
  return 1;
}

void outtake2BallAuton() {
  indexerTop.resetRotation();
  int x = 0;
  while (true) {
    if (fabs(indexerTop.rotation(rev)) < 9) {
      if (ballC == 2) {
        indexer.spin(fwd, 100, pct);
      } else {
        indexerTop.spin(fwd, 100, velocityUnits::pct);
        if (!topConveyor.pressing()) {
          while (x < 200) {
            task::sleep(100);
            x += 100;
          }
          indexerBottom.spin(fwd, 100, pct);
        } else {
          indexerBottom.stop(brake);
        }
      }
    } else {
      indexer.stop(brake);
      break;
    }
    task::sleep(1);
  }
}

int outtake3Ball() {
  indexerTop.resetRotation();
  while (true) {
    if (fabs(indexerTop.rotation(rev)) < 9) {
     indexer.spin(fwd, 100, pct);
    } else {
      indexer.stop(brake);
      ballC -= 3;
      break;
    }
    task::sleep(1);
  }
  return 1;
}

void outtake3BallAuton() {
  indexer.resetRotation();
  while (true) {
    if (indexer.rotation(rev) < 6) {
      indexer.spin(fwd, 100, velocityUnits::pct);
    }

    else {
      indexer.stop(brake);
      break;
    }
    task::sleep(1);
  }
}

int BallCount() {
  while (true) {
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
  while(true){
    if(ballC >= 1){
      brakeIntake();
      printf("balls %i\n", ballC);
    }
    if(topConveyor.pressing()){
      stopIntakeOn();
      break;
    }
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

void center4GoalAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  task::sleep(75);
  moveForwardWalk(34, 90, 0, 1000, false);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  rotatePID(90, 90);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(36, 100, 90, 1000);
  outtake1BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  task::sleep(75);
  moveForwardWalk(-32, 90, 90, 1000);
  rotatePID(-45, 80);
  task::sleep(50);
  createIntakeOnTask();
  moveForwardWalk(25, 90, -45, 1000);
  task::sleep(75);
  stopIntakeOn();
  rotatePID(45, 90);
  moveForwardWalk(-24, 40, 45, 0.6);
  rotatePID(-45, 90);
  moveForwardWalk(-1, 90, -45, 1000);
  rotatePID(-90, 90);
  moveForwardWalk(12, 90, -90, 1000);
  outtake1BallAuton();
  moveForwardWalk(-10, 100, -90, 1000);

}

void centerGoalAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  moveForwardWalk(34, 90, 0, 1000, false);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  rotatePID(90, 90);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(36, 100, 90, 1000);
  outtake1BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  task::sleep(75);
  moveForwardWalk(-32, 90, 90, 1000, false);
  rotatePID(-45, 90);
  createIntakeOnTask();
  moveForwardWalk(24, 90, -45, 1000);
  task::sleep(75);
  stopIntakeOn();
  moveForwardWalk(-22, 100, -45, 0.6);
  task::sleep(75);
  rotatePID(-90, 80);
  intakeRight.spin(fwd, -100, pct);
  intakeLeft.spin(fwd, 100, pct);
  moveForwardWalk(40, 100, -90, 1000);
  outtake1BallAuton();
  brakeIntake();
  moveForwardWalk(-4, 100, -90, 1000, false);
  createIntakeOnTask();
  rotatePID(-170, 90);
  task stop = task(stopIntakeFunction);
  moveForwardWalk(82, 100, -165, 1000);
  outtake1BallAuton();
  task::stop(sto);
  indexer.stop();
  moveForwardWalk(-24, 100, -165, 1000);

}

void homeRowAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  task::sleep(200);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  moveForwardWalk(34, 90, 0, 10);
  rotatePID(90, 100);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(34, 90, 90, 10);
  outtake2BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  moveForwardWalk(-24, 90, 90, 10);
  brakeIntake();
  rotatePID(225, 100);
  moveForwardWalk(82, 90, 225, 10);
  rotatePID(180, 100);
  createIntakeOnTask();
  task st = task(stopIntakeFunction2nd);
  moveForwardWalk(28, 90, 180, 10);
  outtake2BallAuton();
  task::stop(st);
  intake.spin(reverse, 100, pct);
  moveForwardWalk(-24, 90, 180, 10);
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
  moveForwardWalk(24, 90, 0, 10);
  task::sleep(100);
  rotatePID(90, 100);
  moveForwardWalk(24, 90, 90, 10);
  moveForwardWalk(-24, 90, 90, 10);
  task::sleep(100);
  rotatePID(0, 100);
  moveForwardWalk(-24, 90, 0, 10);
}
