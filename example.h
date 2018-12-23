// Example for a drive base with VEX 393 motors
// Written for RobotC

// Base loop flow
// - Motor inputs updated every 20 ms (LOOPTIME)
// - Normal looping until within 100 ticks of position
// - Stop .25 seconds after getting to this position

// Drift correction (autoDrive2)
// - Maintain same encoder values for left and right
// - Second PID loop in addition to position PID
// - Keeps difference between left and right close to 0
// - Position PID is capped at 100
// - Even at max speed, drift PID can correct drift
// - Such as left base at 127, right base at 73

// Slew rate
// - Prevents robot from changing speed too quickly
// - Change capped at 10 per loop (MAXSTEP)

// Should be added
// - Timer to break out of loop if running for too long

#define MAXSTEP 10  // Max change per iteration
#define LOOPTIME 20   // 20 ms, 50 Hz

#include "pidlib.h"

// PID objects with somewhat reasonable constants
// Only P is used
// Not sure if constructors work in this scope
PID basePID(0.5, 0, 0, 0), driftPID(1, 0, 0, 0);

// Sets left base motors
void driveL(int power) {
    motor[left] = power;
}

// Sets right base motors
void driveR(int power) {
    motor[right] = power;
}

// Returns left encoder value
int getLeftEncoder() {
    return SensorValue[leftEncoder];
}

// Returns right encoder value
int getRightEncoder() {
    return SensorValue[rightEncoder];
}

// Clears encoders
void clearEncoders() {
    SensorValue[leftEncoder] = 0;
    SensorValue[rightEncoder] = 0;
}

// Using only left encoder
void autoDrive(int target) {

    bool close = false, done = false;
    long closeTime;
    float driveOut;

    clearEncoders();

    while(!done) {
        
        driveOut = basePID.calculate(target, getLeftEncoder());
        
        driveL(driveOut);
        driveR(driveOut);

        sleep(LOOPSPEED);

        if(abs(target - getLeftEncoder()) < 100) {
            close = true;
            closeTime = getTime();
        }

        if(close && (getTime() - closeTime) > 250) {
            done = true;
            driveL(0);
            driveR(0);
        }

    }

}

// Using both encoders to drive straight
void autoDrive2(int target) {

}

// Using both encoders and slew rate control
void autoDrive3(int target) {

}

void pre_auton() {

}