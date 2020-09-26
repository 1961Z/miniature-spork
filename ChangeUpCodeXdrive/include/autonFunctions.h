#ifndef AUTONFUNCTIONS_H
#define AUTONFUNCTIONS_H

void inertialCalibration();

void setDriveSpeed(int leftSpeed, int rightSpeed);

void holdDrive(); 

void brakeDrive(); 

void coastDrive();

void setIntakeSpeed(int speed);

void brakeIntake(); 

void setConveyorSpeed(int speed);

void brakeConveyor();

void moveForward(int speed, double distanceToTravel);

void moveForwardFast(int speed, double distanceToTravel);

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double multiplyForHorizontal, double addingFactor);

void deaccel(int speed, double dist, double strength);

void strafeWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double addingFactor);

void turnCrawl(double degree, int velocity);  

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

int outtake1Ball( void );

int outtake2Ball( void ); 

int outtake3Ball( void );

void primShooterWithLimit( void );

int bcount(void);

int BallCount(void);

int intakeOn( void );

void intakeOff( void );

void outtake1BallAuton( void );
void outtake2BallAuton( void );
void outtake3BallAuton( void );

void createBallCountTask( void );

void stopBallCountTask( void );

void createPrimeTask( void );

void stopPrimeTask( void );

void createIntakeOnTask( void );

void stopIntakeOn( void );

void homeRowAuton( void );

extern bool whenIntakingPrime; 

extern int counterForSigs; 

#endif  