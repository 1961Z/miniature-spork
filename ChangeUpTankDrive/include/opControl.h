#ifndef OPCONTROL_H
#define OPCONTROL_H

int joyStickControl(void);

int autoAlignWithGoal(void);

int conveyorToggle(void);

int primeTheConveyor(void);

int toggle(void);

int intakeBall(void);

void outtakeAll( void );

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

extern double front_left, front_right, back_left, back_right;

#endif 