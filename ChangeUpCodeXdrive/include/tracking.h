#ifndef TRACKING_H
#define TRACKING_H

int sgn(double num);
double deg_to_rad(double degrees);
double rad_to_deg(double radians);
int update(void);
float average_inertial(void);
float average_inertial_wrapped(void);

void forwardWhileRotating(int angle, double K, double V, double strafeDistance);

void forwardWhileRotating30(int angle, double K, double V, double strafeDistance);

void forwardWhileRotating30to90(int angle, double K, double V, double strafeDistance);

void forwardWhileRotating90to145(int angle, double K, double V, double strafeDistance);

void move_drive(int x, int y, int a);

int move_to_target( void );

void newTryAtTurningWhileGoingForward(int maxSpeed, double startingAngle, double endingAngle);

//void brakeDrive(void);

void move_to_target_sync(double target_x, double target_y, double target_a, bool brakeOn = true, double max_xy = 127, bool cubeLineUp = false,  bool debug = false, bool inDrive = false);

class Tracking {
public:
  double xcoord = 0, ycoord = 0, global_angle = 0, global_angle_radian = 0, power_a = 0, power_x = 0, power_y = 0, x2 = 0 , y2 = 0, a2= 0, holdAngle= 0, driveError  = 0, velocityL = 0, velocityR = 0, velocityB = 0, target_x = 0 , target_y = 0, target_a = 0;
  bool toggle_target, toggle_cube, target, cube, moveComplete = false;
};

struct moveTargetParams {
  double target_x;
  double target_y;
  double target_a;
  bool brakeOn = true;
  double max_xy = 127;
  bool cubeLineUp = false;
  bool debug = false;
  bool inDrive = false;
};


extern Tracking tracking;

#endif