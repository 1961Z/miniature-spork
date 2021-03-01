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

/* 
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
  leftDrive.spin(fwd, directionLeft * 50, pct);
  rightDrive.spin(fwd, directionRight *  50, pct);
  }
  
  task::sleep(90);

 if(distanceTraveled > distanceAtEnd && switchStatement == false && fabs(get_average_inertial()) < angleAtEnd){
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

    /*if(((direction * wheelRevs) - 1.06) > distanceTraveled){
    headingError = -(headingOfRobot - get_average_inertial()) * (multiply);
    }
    
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

#include <iostream>
#include "math.h"

using namespace std;

bool exit_function = false;
double robotAngle;

struct PID{
  float current;
	float kP;
	float kI;
	float kD;
	float target;
	float integral;
	float error;
	float derivative;
	float lastError;
};

PID sRotatePid;

int iRotatePid(int target) {
  sRotatePid.kP = 0.51;
  sRotatePid.kI = 0;
  sRotatePid.kD = 0.32;

  sRotatePid.current = robotAngle;
  sRotatePid.error = target - sRotatePid.current;
  sRotatePid.integral += sRotatePid.error;
  sRotatePid.derivative = sRotatePid.error - sRotatePid.lastError;
  sRotatePid.lastError = sRotatePid.error;

  return (((sRotatePid.error) * (sRotatePid.kP)) + ((sRotatePid.derivative) * (sRotatePid.kD)) + ((sRotatePid.integral) * (sRotatePid.kI)));
}

void rotatePID(int angle, int maxPower) {
  double maxError = 0.1;
  int timer = 0;
  int minVelocity = 10;
  exit_function = false;
  while (fabs(robotAngle - angle) > maxError && !exit_function) {
    robotAngle += 1;
    int PIDPower = iRotatePid(angle);
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    printf("angle  %f\n", robotAngle);
    printf("power  %i\n", power);
  }
}

int main()
{
   rotatePID(90, 90);
}
    */ // code for curves can be added later if needed to be used