#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor front_R = motor(PORT10, ratio6_1, false);
motor front_L = motor(PORT4, ratio6_1, true);
motor middle_R = motor(PORT20, ratio6_1, true);
motor middle_L = motor(PORT12, ratio6_1, false);
motor back_R = motor(PORT8, ratio6_1, false);
motor back_L = motor(PORT6, ratio6_1, true);
motor intakeLeft = motor(PORT4, ratio6_1, false);
motor intakeRight = motor(PORT3, ratio6_1, true);
motor indexerRight = motor(PORT3, ratio6_1, false);
motor indexerLeft = motor(PORT9, ratio6_1, true);
motor conveyor_L = motor(PORT5, ratio6_1, true);
motor conveyor_R =  motor(PORT6, ratio6_1, false);

inertial inertial_Up = inertial(PORT13);
inertial inertial_Down = inertial(PORT14);
line LineTrackerTop = line(Brain.ThreeWirePort.H);
line LineTrackerIntake = line(Brain.ThreeWirePort.F);
line LineTrackerMiddle = line(Brain.ThreeWirePort.G);
limit goalChecker = limit(Brain.ThreeWirePort.A);
encoder verticalTracker = encoder(Brain.ThreeWirePort.A);
encoder horizontalTracker = encoder(Brain.ThreeWirePort.C);
controller Controller1 = controller(primary);

motor_group   leftDrive( front_L, back_L );
motor_group   rightDrive( front_R, back_R );
motor_group   intake( intakeLeft, intakeRight);
motor_group   indexer( indexerLeft, indexerRight); 

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}