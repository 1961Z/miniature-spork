using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor front_L;
extern motor back_L;
extern motor middle_L;
extern motor middle_R;
extern motor front_R;
extern motor back_R;
extern motor intakeLeft;
extern motor intakeRight;
extern motor indexerLeft;
extern motor indexerRight;
extern motor conveyor_R;
extern motor conveyor_L;
extern inertial inertial_Up;
extern inertial inertial_Down;
extern controller Controller1;
extern line LineTrackerTop;
extern line LineTrackerIntake;
extern line LineTrackerMiddle;
extern limit goalChecker; 
extern encoder verticalTracker; 
extern encoder horizontalTracker;
extern motor_group leftDrive; 
extern motor_group intake;
extern motor_group rightDrive;
extern motor_group indexer;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);