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
motor intakeLeft = motor(PORT7, ratio6_1, true);
motor intakeRight = motor(PORT5, ratio6_1, false);
motor indexerRight = motor(PORT3, ratio6_1, false);
motor indexerLeft = motor(PORT9, ratio6_1, true);

inertial inertial_Up = inertial(PORT11);
inertial inertial_Down = inertial(PORT20);
line LineTrackerIntake = line(Brain.ThreeWirePort.F);
limit topConveyor = limit(Brain.ThreeWirePort.E);
encoder leftTracker = encoder(Brain.ThreeWirePort.A);
encoder rightTracker = encoder(Brain.ThreeWirePort.C);
controller Controller1 = controller(primary);

motor_group   leftDrive( front_L, back_L );
motor_group   rightDrive( front_R, back_R );
motor_group   intake( intakeLeft, intakeRight);
motor_group   indexer( indexerLeft, indexerRight); 

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}