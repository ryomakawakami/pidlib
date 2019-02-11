#ifndef PIDLIB
#define PIDLIB

// Returns system time in ms
long getTime() {
    return(nPgmTime);
}

// PID struct
// lastTime stores time of previous calculation
// kP, kI, kD, kF are constants for PID
// lastValue stores the last value of the input
// sigma stores the total error
typedef struct {
    long lastTime;
    float kP, kI, kD, kF;
    float lastValue;
    float sigma;
} PID;

// PID function setConstants
// Allows user to input kP, kI, kD and initializes other variables
// lastTime set to current time
// kP, kI, kD, kF set to inputted values
// lastValue, sigma set to 0
void setConstantsPID(PID* pid, float p, float i, float d, float f) {
    lastTime = getTime();
    pid->kP = p;
    pid->kI = i;
    pid->kD = d;
    pid->kF = f;
    pid->lastValue = 0;
    pid->sigma = 0;
}

// PID function calculate
// Calculates control loop output
// Integral range default is a large value
float calculatePID(PID* pid, float target, float sensorValue, float range = 10000) {
    // Declare variables
    float deltaTime, error, derivative, output;

    // Find change in time and store current
    deltaTime = (float)(getTime() - pid->lastTime) / 1000.0;
    pid->lastTime = getTime();

    // Calculate error (P)
    error = target - sensorValue;

    // Calculate sigma (I)
    pid->sigma += error * deltaTime;

    // Reset sigma if outside of integral range
    if(abs(error) > range) {
        pid->sigma = 0;
    }

    // Also reset if robot shoots over
    else if(target > 0) {
        if(error < 0) {
            pid->sigma = 0;
        }
    }
    else {
        if(error > 0) {
            pid->sigma = 0;
        }
    }

    // Calculate derivative (D)
    // Change in value over change in time and store current
    derivative = (sensorValue - pid->lastValue) / deltaTime;
    pid->lastValue = sensorValue;

    // Calculate output
    output = pid->kP * error + pid->kI * sigma + pid->kD * derivative + pid->kF * target;

    return output;
}

#endif