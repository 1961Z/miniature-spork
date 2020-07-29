#include "vex.h"
#include "autonFunctions.h"
#include "vex_timer.h"
#include "tracking.h"
#include "util.h"

using namespace vex;
 
 vex::task *moveTask;
 vex::task *updateTask;

timer Timer; 

moveTargetParams moveParams;

float average_inertial() {
  int robotDirection = -(-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
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

void moveStartTask() {
  task moveTask = task(move_to_target);
}


void moveStopTask() {
  if(moveTask != nullptr)
  {
    delete moveTask;
    moveTask = nullptr;
  }
}

int update (void){
   double distance_B = 3.75;
   double distance_R_to_Center = 2; 
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
   double theta_last = 0; 
   double new_theta = 0;

 while (true) {
//amount encoders moved (radians)
	 newright = -verticalTracker.rotation(deg) / 360.0* (2.85*M_PI);
	 newback =  horizontalTracker.rotation(deg) / 360.0* (2.75*M_PI);
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
   new_theta = deg_to_rad(average_inertial());
	 theta = new_theta - theta_last; 
   theta_last = new_theta;

   
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
     Brain.Screen.printAt(1, 20, "Xcord: %f", tracking.xcoord);
     Brain.Screen.printAt(1, 40, "Ycord: %f", tracking.ycoord);
     Brain.Screen.printAt(1, 60, "Theta: %f", tracking.global_angle);
     /*printf("Xcord: %f\n", tracking.xcoord);
     printf("Ycord: %f\n", tracking.ycoord);
     printf("Theta: %f\n", tracking.global_angle);*/
     
 task::sleep(1);
}
}

double stopWatch = 0;
double angleError = 0;
double switchSpeed = 0; 
double x = 0;
double front_R_speed = 0, front_L_speed = 0, back_R_speed = 0, back_L_speed = 0; 
void forwardWhileRotating(int angle, double K, double V, double strafeDistance) {
  while ((angle - fabs(average_inertial())) > 0) {
    angleError = deg_to_rad(angle - fabs(average_inertial()));
    x = average_inertial(); 
    switchSpeed = -(V * (cos(2 * angleError) - K * (sin(2 * angleError))));
    front_L_speed = -7.507133882 * (pow(10, -6) * pow(x, 4)) - 1.654567633 * (pow(10, -3) * pow(x, 3)) - 1.171957926 * (pow(10, -1) * pow(x, 2)) - (2.361224952 * x) + 22.5403336; //40 * exp(-(pow(x +8.8, 2) /  1000)); 
    front_R_speed = -6.617784206 * (pow(10, -6) * pow(x, 4)) -9.033314118 * (pow(10, -4) * pow(x, 3)) - 3.153343133 * (pow(10, -2) * pow(x, 2)) + 7.184563656 *(pow(10, -2) * x) + 37.84368732; //1/1000 * (pow(x + 23, 3)) + 30;
    back_R_speed = -2.125800572 * (pow(10, -5) * pow(x, 4)) - 3.721159207 * (pow(10, -3) * pow(x, 3)) - 2.233360857 * (pow(10, -1) * pow(x, 2)) - (5.072185833 * x) + 34.02611327; //-1/36 * (pow(x + 40, 2)) + 70; 
    back_L_speed = 5.692069854 * (pow(10, -6) * pow(x, 4)) + 8.673488361 * (pow(10, -4) * pow(x, 3)) + 5.425334863 * (pow(10, -2) * pow(x, 2)) + (2.297187036 * x) + 25.01477343; //1/50 * (pow(x + 51, 2)) - 30; 
      front_L.spin(fwd, front_L_speed, pct);
      front_R.spin(fwd, front_R_speed, pct);
      back_L.spin(fwd, back_L_speed, pct);
      back_R.spin(fwd, back_R_speed, pct);
    printf("angle Error %f\n", angleError);
    printf("angle %f\n", x);
    printf("switchSpeed %f\n", switchSpeed);
    task::sleep(10);
    stopWatch += 0.01;
  }
  brakeDrive();
  strafeWalk(strafeDistance, V, angle, 0.6, 0);
}

void forwardWhileRotating30(int angle, double K, double V, double strafeDistance){
  while ((angle - fabs(average_inertial())) < 0) {
    angleError = deg_to_rad(angle - fabs(average_inertial()));
    x = fabs(average_inertial()); 
    switchSpeed = -(V * (cos(2 * angleError) - K * (sin(2 * angleError))));
    front_L_speed = 2.532569556 * (pow(10, -4) * pow(x, 4)) - 8.120956537 * (pow(10, -3) * pow(x, 3)) + 1.118529929 * (pow(10, -1) * pow(x, 2)) - (1.775451839 * x) - 30.73812133; //40 * exp(-(pow(x +8.8, 2) /  1000)); 
    front_R_speed = -3.153646108 * (pow(10, -4) * pow(x, 4)) + 1.150978513 * (pow(10, -2) * pow(x, 3)) - 1.437346043 * (pow(10, -1) * pow(x, 2)) + (1.277999598 * x) + 33.3360317; //1/1000 * (pow(x + 23, 3)) + 30;
    back_R_speed = 3.364653549 * (pow(10, -4) * pow(x, 4)) - 2.477915471 * (pow(10, -2) * pow(x, 3)) + 5.857538426 * (pow(10, -1) * pow(x, 2)) - (3.842224646 * x) - 15.31392947; //-1/36 * (pow(x + 40, 2)) + 70; 
    back_L_speed = -4.990617896 * (pow(10, -4) * pow(x, 4)) + 3.40428298 * (pow(10, -2) * pow(x, 3)) - 7.252974832 * (pow(10, -1) * pow(x, 2)) + (3.981376812 * x) + 17.96092271; //1/50 * (pow(x + 51, 2)) - 30; 
      front_L.spin(fwd, front_L_speed, pct);
      front_R.spin(fwd, front_R_speed, pct);
      back_L.spin(fwd, back_L_speed, pct);
      back_R.spin(fwd, back_R_speed, pct);
    printf("angle Error %f\n", x);
    printf("angle %f\n", angle - fabs(average_inertial()));
    printf("switchSpeed %f\n", front_L_speed);
    task::sleep(10);
    stopWatch += 0.01;
  }
  brakeDrive();
  strafeWalk(strafeDistance, V, angle, 0.6, 0);
}

void forwardWhileRotating30to90(int angle, double K, double V, double strafeDistance){
  if(rad_to_deg(tracking.global_angle) != -30){
      tracking.global_angle = deg_to_rad(-31);
    }  
  while (front_R_speed == fabs(front_R_speed)) {
    angleError = deg_to_rad(angle - fabs(average_inertial()));
    x = rad_to_deg(tracking.global_angle);
    switchSpeed = -(V * (cos(2 * angleError) - K * (sin(2 * angleError))));
    front_L_speed = -6.069603238 * (pow(10, -7) * pow(x, 5)) - 1.21453075 * (pow(10, -4) * pow(x, 4)) - 7.592845481 * (pow(10, -3) * pow(x, 3)) - 5.337382502 * (pow(10, -2) * pow(x, 2)) + (10.84867401 * x) + 253.3639582; //40 * exp(-(pow(x +8.8, 2) /  1000)); 
    back_L_speed = 3.269079149 * (pow(10, -7) * pow(x, 5)) + 9.448031142 * (pow(10, -5) * pow(x, 4)) + 1.105422826 * (pow(10, -2) * pow(x, 3)) + 0.675634017 * (pow(x, 2)) + (21.67506939 * x) + 270.2324163; //1/50 * (pow(x + 51, 2)) - 30; 
    front_R_speed = 1.734766309 * (pow(10, -7) * pow(x, 5)) + 2.107778829 * (pow(10, -5) * pow(x, 4)) - 6.553838746 * (pow(10, -4) * pow(x, 3)) - 2.102974813 * (pow(10, -1) * pow(x, 2)) - (11.80380785 * x) - 195.1624189; //1/1000 * (pow(x + 23, 3)) + 30;
    back_R_speed = -3.199569272 * (pow(10, -6) * pow(x, 5)) - 9.056022434 * (pow(10, -4) * pow(x, 4)) - 9.947826494 * (pow(10, -2) * pow(x, 3)) - 5.310079135 * (pow(x, 2)) - (137.7852031 * x) - 1385.826421; //-1/36 * (pow(x + 40, 2)) + 70; 
      front_L.spin(fwd, front_L_speed, pct);
      front_R.spin(fwd, front_R_speed, pct);
      back_L.spin(fwd, back_L_speed, pct);
      back_R.spin(fwd, back_R_speed, pct);
    printf("angle Error %f\n", angle - average_inertial());
    printf("frontL %f\n", front_L_speed);
    printf("frontR %f\n", front_R_speed);
    printf("backL %f\n", back_L_speed);
    printf("backR %f\n", back_R_speed);
    task::sleep(10);
    stopWatch += 0.01;
  }
  //brakeDrive();
  strafeWalk(strafeDistance, V, angle, 0.6, 0);
}

void forwardWhileRotating90to145(int angle, double K, double V, double strafeDistance){
  if(rad_to_deg(tracking.global_angle) != -90){
      tracking.global_angle = deg_to_rad(-90);
    }  
  while (front_L_speed != fabs(front_L_speed)) {
    angleError = deg_to_rad(angle - fabs(average_inertial()));
    x = rad_to_deg(tracking.global_angle);
    switchSpeed = -(V * (cos(2 * angleError) - K * (sin(2 * angleError))));
    front_L_speed = -2.831065693 * (pow(10, -7) * pow(x, 5)) - 6.054502592 * (pow(10, -6) * pow(x, 4)) + 3.462686972 * (pow(10, -2) * pow(x, 3)) + 8.132728517 * (pow(x, 2)) + (706.8975916 * x) + 21714.42148; //40 * exp(-(pow(x +8.8, 2) /  1000)); 
    back_L_speed = 2.614845958 * (pow(10, -7) * pow(x, 5)) + 1.098729194 * (pow(10, -4) * pow(x, 4)) + 1.592589554 * (pow(10, -2) * pow(x, 3)) + 7.448158711 * (pow(10, -1) * pow(x, 2)) - (21.11641979 * x) - 1989.140396; //1/50 * (pow(x + 51, 2)) - 30; 
    front_R_speed = -5.336051576 * (pow(10, -6) * pow(x, 5)) - 3.178871586 * (pow(10, -3) * pow(x, 4)) - 7.528523603 * (pow(10, -1) * pow(x, 3)) - 88.68938738 * (pow(x, 2)) - (5202.034033 * x) - 121571.5542; //1/1000 * (pow(x + 23, 3)) + 30;
    back_R_speed = 1.36716667 * (pow(10, -6) * pow(x, 5)) + 7.74599643 * (pow(10, -4) * pow(x, 4)) + 1.741519727 * (pow(10, -1) * pow(x, 3)) + 19.43096109 * (pow(x, 2)) + (1076.358783 * x) + 23690.12506; //-1/36 * (pow(x + 40, 2)) + 70; 
      front_L.spin(fwd, front_L_speed, pct);
      front_R.spin(fwd, front_R_speed, pct);
      back_L.spin(fwd, back_L_speed, pct);
      back_R.spin(fwd, back_R_speed, pct);
    printf("angle Error %f\n", angle - average_inertial());
    printf("frontL %f\n", front_L_speed);
    printf("frontR %f\n", front_R_speed);
    printf("backL %f\n", back_L_speed);
    printf("backR %f\n", back_R_speed);
    task::sleep(10);
    stopWatch += 0.01;
  }
  //brakeDrive();
  moveForwardWalk(strafeDistance, V, angle, 0.6, 0.6, 0);
}

void brakeDrive(){
front_L.stop(hold);
front_R.stop(hold);
back_L.stop(hold);
back_R.stop(hold);
}


void move_drive(int x, int y, int a){

  front_L.spin(fwd, x + y + a, voltageUnits::volt);
  front_R.spin(fwd, -x + y - a, voltageUnits::volt);
  back_L.spin(fwd, -x + y + a, voltageUnits::volt);
  back_R.spin(fwd, x + y - a, voltageUnits::volt);

}

int offset = 0; 

int move_to_target(){
  tracking.target_x = moveParams.target_x;
  tracking.target_y = moveParams.target_y;
  tracking.target_a = moveParams.target_a;
  double power_d;
  bool debug = moveParams.debug;
  bool cubeLineUp = moveParams.cubeLineUp;
  bool brakeOn = moveParams.brakeOn;
  bool inDrive = moveParams.inDrive;
  double x = 10; 
  double max_power_a = x, max_power_xy = moveParams.max_xy;
  double min_power_a = 1, min_power_xy = 2;
  double scale;

  double last_power_a = max_power_a, last_power_x = max_power_xy, last_power_y = max_power_xy;
  double integral_a = 0.0, integral_d = 0.0;

  double error_a, error_x, error_y, error_d, errorLocalx, errorLocaly;
  double difference_a;
  double kP_a = 127, kP_d = 0.0022;
  double kI_a = 12, kI_d = 0.01;   // kI_a = 0.01, kI_d = 0.0022;
  unsigned long last_time = timer().time(msec);

  error_a = tracking.target_a - tracking.global_angle;
  error_x = tracking.target_x - tracking.xcoord;
  error_y = tracking.target_y - tracking.ycoord;
  error_d = sqrtf(powf(error_x, 2) + powf(error_y, 2));

  while (inDrive == false){


    error_a = tracking.target_a - tracking.global_angle;
    error_x = tracking.target_x - tracking.xcoord;
    error_y = tracking.target_y - tracking.ycoord;
    error_d = sqrtf(powf(error_x, 2) + powf(error_y, 2));
    tracking.driveError = error_d;

    difference_a = tracking.global_angle + atan(error_y/error_x);
    errorLocalx = cos(difference_a) * error_d;
    errorLocaly = sin(difference_a) * error_d;


    if (fabs(error_a) < 5){
      integral_a += error_a * (timer().time(msec)-last_time);
    }

    if (fabs(error_d) < 2.5){ // what triggers integral to start adding?
      integral_d += error_d * (timer().time(msec)-last_time);
    }
    if(fabs(error_d)<=0.7) {
      integral_d = 0;
    }

    if(fabs(error_a)> deg_to_rad(0.5)){
      tracking.power_a = map_set(fabs(error_a),deg_to_rad(0.5), M_PI,12.0*sgn(error_a),127.0*sgn(error_a),
                        deg_to_rad(5), sgn(error_a)*20.0,
                        deg_to_rad(20),sgn(error_a)*50.0,
                        deg_to_rad(25),sgn(error_a)*70.0,
                        deg_to_rad(45),sgn(error_a)*85.0,
                        deg_to_rad(60), sgn(error_a)*110.0,
                        deg_to_rad(90),sgn(error_a)*120.0,
                        M_PI,sgn(error_a)*127.0);
    }
    else {
      tracking.power_a = 0;
    }

    power_d = map_set(fabs(error_d),0.5,200.0,15.0,127.0,
                      10.0,127.0,
                      15.0,127.0,
                      18.0, 127.0,
                      25.0,127.0,
                      200.0,127.0);

    if (error_x >= 0){
      tracking.power_x = error_d*cos(difference_a)*kP_d;
      tracking.power_y = error_d*sin(difference_a)*kP_d;
    }
    else{
      tracking.power_x = -error_d*cos(difference_a)*kP_d;
      tracking.power_y = -error_d*sin(difference_a)*kP_d;
    }
    tracking.power_x+= sgn(tracking.power_x) * integral_d * kI_d;
    tracking.power_y+= sgn(tracking.power_y) * integral_d * kI_d;
//controlling max power for a, x, and y
    if (fabs(tracking.power_a) > max_power_a){
      tracking.power_a = sgn(tracking.power_a)*max_power_a;
    }
    // need to scale x and y powers
    if (fabs(tracking.power_x) > max_power_xy || fabs(tracking.power_y) > max_power_xy){
      if (fabs(tracking.power_x) > fabs(tracking.power_y)){
        scale = max_power_xy/fabs(tracking.power_x);
        tracking.power_x = max_power_xy *sgn(tracking.power_x);
        tracking.power_y = tracking.power_y *scale;
        tracking.power_a = tracking.power_a*scale;
      }
      else {
        scale = max_power_xy/fabs(tracking.power_y);
        tracking.power_y = max_power_xy *sgn(tracking.power_y);
        tracking.power_x = tracking.power_x *scale;
        tracking.power_a = tracking.power_a*scale;
      }
    }

    if (fabs(errorLocalx) > 0.5 || fabs(error_x) > 0.5 ){
        if (fabs(tracking.power_x) < min_power_xy){
        tracking.power_x = sgn(tracking.power_x)*min_power_xy;
      }
    }
    else {
      tracking.power_x = 0;
    }


    if (fabs(errorLocaly) > 0.5 || fabs(error_y) > 0.5){
      if(fabs(tracking.power_y) < min_power_xy){

      tracking.power_y = sgn(tracking.power_y)*min_power_xy;
      }
    }
    else{
      tracking.power_y = 0;
    }
    
    printf("xspeed: %f\n", tracking.power_x);
    printf("Yspeed: %f\n", tracking.power_y);
    printf("Theataspeed: %f\n", tracking.power_a);
    printf("error_D %f\n", error_d);

    move_drive(tracking.power_x, tracking.power_y, tracking.power_a);
    if(tracking.power_x != 0) last_power_x = tracking.power_x;
    if(tracking.power_y != 0) last_power_y = tracking.power_y;
    if(tracking.power_a != 0) last_power_a = tracking.power_a;
    last_time = timer().time(msec); 
    if (fabs(error_a) <= deg_to_rad(0.5) && fabs(error_d) < 2 && cubeLineUp){
      if(cubeLineUp){
        printf("I'm doing a cube line up\n");
        //green.linedUp = false;
        while((fabs(error_a) >= deg_to_rad(0.5)) || (fabs(error_y) >= 0.5 )) {
          error_y =tracking.target_y - tracking.ycoord;
          error_a = fmod(tracking.target_a - tracking.global_angle, 2*M_PI);
          //green.lineMiddle(0.8,tracking.target_y,tracking.target_a);
          move_drive(tracking.power_x, tracking.power_y, tracking.power_a);
        }
        offset = tracking.target_x - tracking.xcoord;
        move_drive(0, 0, 0);
        difference_a = 0;
        printf("i can reach here");
        tracking.moveComplete = true;
        moveStopTask();
        break;
        }
    }
    else if (fabs(error_a) <= deg_to_rad(0.5) && fabs(error_x) <= 0.5 && fabs(error_y) <= 0.5 && !cubeLineUp){
      difference_a = 0;
      printf("i hope");
      brakeDrive();
      if(brakeOn) {
        task::sleep(600);
      }
      tracking.moveComplete = true;
      moveStopTask();
      break;
    }
    task::sleep(10);
  }
  return 1;
}

void move_to_target_sync(double target_x, double target_y, double target_a, bool brakeOn, double max_xy, bool cubeLineUp,  bool debug, bool inDrive) {
  if(!tracking.moveComplete) moveStopTask();
  if(moveTask != nullptr) moveTask = nullptr;
  moveParams.target_x = target_x;
  moveParams.target_y = target_y;
  moveParams.target_a = target_a; 
  moveParams.brakeOn = brakeOn; 
  moveParams.max_xy = max_xy; 
  moveParams.cubeLineUp = cubeLineUp; 
  moveParams.debug = debug; 
  moveParams.inDrive = inDrive;
  tracking.driveError = 0;
  tracking.moveComplete = false;
  move_to_target();
}
