#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor front_L = motor(PORT12, ratio18_1, false);
motor front_R = motor(PORT19, ratio18_1, true);
motor back_R = motor(PORT20, ratio18_1, true);
motor back_L = motor(PORT11, ratio18_1, false);
motor intake_R = motor(PORT9, ratio18_1, true);
motor intake_L = motor(PORT4, ratio18_1, false);
motor conveyor_L = motor(PORT3, ratio6_1, true);
motor conveyor_R =  motor(PORT18, ratio6_1, false);
inertial inertial_Up = inertial(PORT1);
inertial inertial_Down = inertial(PORT10);
line LineTrackerTop = line(Brain.ThreeWirePort.B);
line LineTrackerIntake = line(Brain.ThreeWirePort.A);
line LineTrackerMiddle = line(Brain.ThreeWirePort.C);
limit goalChecker = limit(Brain.ThreeWirePort.H);
encoder verticalTracker = encoder(Brain.ThreeWirePort.E);
encoder horizontalTracker = encoder(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}