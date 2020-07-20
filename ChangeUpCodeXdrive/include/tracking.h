#ifndef TRACKING_H
#define TRACKING_H

int sgn(double num);
double deg_to_rad(double degrees);
double rad_to_deg(double radians);
int update(void);
float average_inertial(void);

class Tracking {
public:
  double xcoord = 0, ycoord = 0, global_angle = 0, power_a = 0, power_x = 0, power_y = 0, x2 = 0 , y2 = 0, a2= 0, holdAngle= 0, driveError  = 0, velocityL = 0, velocityR = 0, velocityB = 0, target_x = 0 , target_y = 0, target_a = 0;
};

extern Tracking tracking;

#endif