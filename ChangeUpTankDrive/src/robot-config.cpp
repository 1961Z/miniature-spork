#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor front_R = motor(PORT10, ratio6_1, false);
motor front_L = motor(PORT4, ratio6_1, true);
motor middle_R = motor(PORT9, ratio6_1, true);
motor middle_L = motor(PORT5, ratio6_1, false);
motor back_R = motor(PORT8, ratio6_1, false);
motor back_L = motor(PORT6, ratio6_1, true);
motor intake = motor(PORT7, ratio6_1, true);
motor indexer = motor(PORT3, ratio6_1, false);
motor conveyor_L = motor(PORT1, ratio6_1, true);
motor conveyor_R =  motor(PORT2, ratio6_1, false);

inertial inertial_Up = inertial(PORT11);
inertial inertial_Down = inertial(PORT20);
line LineTrackerTop = line(Brain.ThreeWirePort.D);
line LineTrackerIntake = line(Brain.ThreeWirePort.B);
line LineTrackerMiddle = line(Brain.ThreeWirePort.C);
limit goalChecker = limit(Brain.ThreeWirePort.A);
encoder verticalTracker = encoder(Brain.ThreeWirePort.E);
encoder horizontalTracker = encoder(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);

motor_group   leftDrive( front_L, middle_L, back_L );
motor_group   rightDrive( front_R, middle_R, back_R );

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}