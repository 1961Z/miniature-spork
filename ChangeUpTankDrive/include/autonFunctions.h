#ifndef AUTONFUNCTIONS_H
#define AUTONFUNCTIONS_H

void inertialCalibration();

void setDriveSpeed(int leftSpeed, int rightSpeed);

void holdDrive(); 

int debugging( void );

void brakeDrive(); 

void coastDrive();

void setIntakeSpeed(int speed);

void brakeIntake(); 

void setConveyorSpeed(int speed);

void brakeConveyor();

void moveForward(int speed, double distanceToTravel);

void moveForwardFast(int speed, double distanceToTravel);

double calculateLeftSpeed(double speed, int turningRadius);

double calculateRightSpeed(double speed, int turningRadius);

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double multiplyForHorizontal, double addingFactor, int sideWays, double turningRadius,  double angleNeeded, double sideWaysDistance, double stafeAtEnd, double distanceAtEnd, double angleAtEnd, double turningRadiusAtEnd);

void deaccel(int speed, double dist, double strength);

void strafeWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double addingFactor);

void stafeThanForward(double speed, bool side);

void turnCrawl(double degree, int velocity);  

void rightPivotTurn(int speed, int angle, double turningRadius);

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

int iMovePid(int target);

int iRotatePid(int target);

int intakeToggle(void);

void wait_until_drive_settled(int angle);

void rotatePID(int angle);

void rotatePID(int angle, int maxPower); 

void resetFunction();

void goTo(int sigNumber, int velocity);

void ObjectLooker(int sigNumber, int speed);

void strafeSimpleRight(int speed);

void strafeSimpleLeft(int speed);

void moveForwardSimple(int speed);

void strafeWhileTurning(int speed, double distance); 

int primeShoot( void ); 

int goBackDown( void );

void intakeMoves( void );

int scoreGoal( void );

void primeShooterWithVision( void );

int outtake0Ball( void );

int outtake1Ball( void );

int outtake2Ball( void ); 

int outtake3Ball( void );

void primShooterWithLimit( void );

int bcount(void);

int BallCount(void);

int intakeOn( void );

void intakeOff( void );

void outtake1BallAuton( void );
void outtake1BallAutonSlow( void );
void outtake2BallAuton( void );
void outtake3BallAuton( void );

void createBallCountTask( void );

void stopBallCountTask( void );

void createPrimeTask( void );

void stopPrimeTask( void );

void createIntakeOnTask( void );

void stopIntakeOn( void );

void outtakeIntakes(double revolutions, int speed);

void preAuton( void );

void ball2Auton( void );

void skills( void );

void testAuton( void );

void homeRowAuton( void );

extern bool whenIntakingPrime; 

extern int counterForSigs; 

#endif  