#ifndef PIDLIB
#define PIDLIB

// Returns system time in ms
long getTime() {
    return(nPgmTime);
}

// PID class
// lastTime stores time of previous calculation
// kP, kI, kD are constants for PID
// lastError stores the last value of the error
// sigma stores the total error
class PID {
    public:
        PID PID(float p, float i, float d);
        void setConstants(float p, float i, float d);
        float calculate(float target, float sensorValue, float range = 10000);
    private:
        long lastTime;
        float kP, kI, kD;
        float lastError;
        float sigma;
};

// PID object constructor
// lastTime set to current time
// kP, kI, kD set to inputted values
// lastError, sigma set to 0
PID::PID(float p, float i, float d) {
    lastTime = getTime();
    kP = p;
    kI = i;
    kD = d;
    lastError = 0;
    sigma = 0;
}

// PID function setConstants
// Allows user to input kP, kI, kD and initializes other variables
// lastTime set to current time
// kP, kI, kD set to inputted values
// lastError, sigma set to 0
void PID::setConstants(float p, float i, float d) {
    lastTime = getTime();
    kP = p;
    kI = i;
    kD = d;
    lastError = 0;
    sigma = 0;
}

// PID function calculate
// Calculates control loop output
// Integral range default is a large value
float PID::calculate(float target, float sensorValue, float range = 10000) {
    // Declare variables
    float deltaTime, error, derivative, output;

    // Find change in time and store current
    deltaTime = (float)(getTime() - lastTime) / 1000.0;
    lastTime = getTime();

    // Calculate error (P)
    error = target - sensorValue;

    // Calculate sigma (I)
    sigma += error * deltaTime;

    // Reset sigma if outside of integral range
    if(abs(error) > range) {
        sigma = 0;
    }

    // Also reset if robot shoots over
    else if(target > 0) {
        if(error < 0) {
            sigma = 0;
        }
    }
    else {
        if(error > 0) {
            sigma = 0;
        }
    }

    // Calculate derivative (D)
    // Change in value over change in time and store current
    derivative = (error - lastError) / deltaTime;
    lastError = error;

    // Calculate output
    output = kP * error + kI * sigma + kD * derivative;

    return output;
}

#endif