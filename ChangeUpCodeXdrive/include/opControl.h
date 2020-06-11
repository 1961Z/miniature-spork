#ifndef OPCONTROL_H
#define OPCONTROL_H

int joyStickControl(void);

int autoAlignWithGoal(void);

int intakeToggle(void);

int conveyorToggle(void);

extern double front_left, front_right, back_left, back_right;

#endif 