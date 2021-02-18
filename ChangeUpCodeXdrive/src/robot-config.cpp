#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor front_L = motor(PORT13, ratio18_1, false);
motor front_R = motor(PORT9, ratio18_1, true);
motor back_L = motor(PORT12, ratio18_1, false);
motor back_R = motor(PORT8, ratio18_1, true);
motor intake_L = motor(PORT15, ratio6_1, false);
motor intake_R = motor(PORT16, ratio6_1, true);
motor conveyor_L = motor(PORT4, ratio6_1, true);
motor conveyor_R =  motor(PORT10, ratio6_1, false);
inertial inertial_Up = inertial(PORT11);
inertial inertial_Down = inertial(PORT20);
line LineTrackerTop = line(Brain.ThreeWirePort.D);
line LineTrackerIntake = line(Brain.ThreeWirePort.B);
line LineTrackerMiddle = line(Brain.ThreeWirePort.C);
limit goalChecker = limit(Brain.ThreeWirePort.A);
encoder verticalTracker = encoder(Brain.ThreeWirePort.E);
encoder horizontalTracker = encoder(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}