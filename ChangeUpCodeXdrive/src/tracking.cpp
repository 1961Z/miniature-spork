#include "vex.h"
#include "vex_timer.h"
#include "tracking.h"

using namespace vex;

timer Timer; 

float average_inertial() {
  float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
  return robotDirection;
}

float average_inertial_wrapped() {
  float robotDirection = (-inertial_Up.heading(deg) - inertial_Down.heading(deg)) / 2;
  return robotDirection;
}

double deg_to_rad(double degrees){
  return degrees/180 *M_PI;
}

double rad_to_deg(double radians){
  return radians/M_PI *180;
}

int sgn(double num){
  if (num < 0){
    return -1;
  }
  else if (num > 0){
    return 1;
  }
  else{
    return 0;
  }
}

int update (void){
   double distance_B = 7.25;
   double distance_R_to_Center = 6; 
   double radiusR = 0;
   double radiusB = 0;
   double h = 0;
   double h2 = 0;
   double theta = 0; double beta = 0; double alpha = 0; double theta_wrapped = 0; 
   double Xx = 0; double Xy = 0; double Yx = 0; double Yy = 0;
   double newright = 0; double newback = 0;
   double Right = 0; double Back = 0;
   double lastright = 0, lastback = 0;
   uint32_t last_time = 0;
   double rightLastVel=0; double backLastVel = 0;
   double timeUpdate = 0;

 while (true) {
//amount encoders moved (radians)
	 newright = verticalTracker.rotation(deg) / 360.0* (2.75*M_PI);
	 newback = horizontalTracker.rotation(deg) / 360.0* (2.75*M_PI);
	 Right = newright - lastright;
	 Back = newback - lastback;
   timeUpdate = timer().time(msec)-last_time; 

   if(timeUpdate >= 20) {
     tracking.velocityR = (newright - rightLastVel)/(timeUpdate);
     tracking.velocityB = (newback - backLastVel)/(timeUpdate);
     rightLastVel = newright;
     backLastVel = newback;
     last_time = timer().time(msec);
   }
//update last
	 lastright = newright;
	 lastback = newback;
	 theta = average_inertial();
   
//if robot turned in any direction
	 if (theta != 0){
		 radiusR = Right / theta;
		 beta = theta / 2.0;
		 h = (radiusR + distance_R_to_Center) * 2 *sin(beta);
		 radiusB = Back / theta;
		 h2 = (radiusB + distance_B) * 2 *sin(beta);
	 }
//if robot moved straight or didn't move
	 else {
		 h = Right;
		 h2 = Back;
		 beta = 0;
	 }
		 alpha = tracking.global_angle + beta;
//update global x, y and angle
		 Xx = h2 * cos(alpha);
		 Xy = h2 * -sin(alpha);
		 Yx = h * sin(alpha);
		 Yy = h * cos(alpha);
		 tracking.xcoord += Yx + Xx;
		 tracking.ycoord += Yy + Xy;
     tracking.global_angle += theta;
     tracking.global_angle_radian += deg_to_rad(theta_wrapped);
 task::sleep(1);
}
}

void forwardWhileRotating(int xdistance, int angle){
double xToTravel = 0; 
double yToTravel = 0; 
double lConstant = 0; 

xToTravel = (tracking.velocityR * cos(angle)) - (lConstant * tracking.global_angle_radian * sin(angle)); 
yToTravel = (tracking.velocityB * sin(angle)) + (lConstant * tracking.global_angle_radian * cos(angle));


}