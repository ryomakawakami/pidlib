#ifndef PIDLIB
#define PIDLIB

// Check logic
// Add integral bounds

// Returns system time in ms
long getTime() {
    return(nPgmTime);
}

// PID class
class PID {
    public:
        PID PID(float p, float i, float d);
        void setConstants(float p, float i, float d);
        float calculate(float target, float sensorValue);
    private:
        long lastTime;
        float kP, kI, kD;
        float lastError;
        float sigma;
};

// Constructor
PID::PID(float p, float i, float d) {
    lastTime = getTime();
    kP = p;
    kI = i;
    kD = d;
    lastError = 0;
    sigma = 0;
}

// Set kP, kI, kD
void PID::setConstants(float p, float i, float d) {
    lastTime = getTime();
    kP = p;
    kI = i;
    kD = d;
    lastSensorValue = 0;
    sigma = 0;
}

// Calculate control loop output
float PID::calculate(float target, float sensorValue) {
    // Declare variables
    float deltaTime, error, derivative, output;

    // Find change in time and store current
    deltaTime = (getTime() - lastTime) / 1000;
    lastTime = getTime();

    // Calculate error (P)
    error = target - sensorValue;

    // Calculate sigma (I)
    sigma += error * deltaTime;

    // Calculate derivative (D)
    // Change in value over change in time and store current
    derivative = (error - lastError) / deltaTime;
    lastError = error;

    // Calculate output
    output = kP * error + kI * sigma + kD * derivative;

    return output;
}

#endif