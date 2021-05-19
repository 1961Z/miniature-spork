#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor front_R = motor(PORT20, ratio6_1, true);
motor front_L = motor(PORT12, ratio6_1, false);
motor back_R = motor(PORT19, ratio6_1, false);
motor back_L = motor(PORT11, ratio6_1, true);
motor intakeLeft = motor(PORT7, ratio6_1, true);
motor intakeRight = motor(PORT8, ratio6_1, false);
motor indexerBottom = motor(PORT5, ratio6_1, true);
motor indexerTop = motor(PORT6, ratio6_1, false);

inertial inertial_Up = inertial(PORT15);
inertial inertial_Down = inertial(PORT16);
encoder rightTracker = encoder(Brain.ThreeWirePort.A); //flipped 
encoder leftTracker = encoder(Brain.ThreeWirePort.C);
line LineTrackerIntake = line(Brain.ThreeWirePort.E);
line goalChecker = line(Brain.ThreeWirePort.G);
limit trackerOuttake = limit(Brain.ThreeWirePort.H);
limit topConveyor = limit(Brain.ThreeWirePort.F);

motor_group   leftDrive( front_L, back_L );
motor_group   rightDrive( front_R, back_R );
motor_group   intake( intakeLeft, intakeRight);
motor_group   indexer( indexerBottom, indexerTop); 

controller Controller1 = controller(primary);

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}