#include "klevebrand_maxjet_drone.h"

void KlevebrandMaxJetDrone::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    if (!Serial)
    {
        Serial.println("FAILED TO START SERIAL...");
    }
    while (!Serial)
        ;

    Serial.println("STARTING DRONE...");

    gyro.setup();

    eeprom_pid_repository.setup();

    setupMotors();

    setFlightModeAcro();

    Serial.println("DRONE STARTED!");
}

void KlevebrandMaxJetDrone::run()
{
    long start_micros_timestamp = micros();

    // Get the latest data from the gyroscope
    updateGyro();

    if (hasLostConnection())
    {
        // If connection is dead, stop the drone
        resetPid();
        stopMotors();

        Serial.println("LOST CONNECTION");
    }
    else if (!isMotorsEnabled())
    {
        // If the motors are diabled, stop the drone
        resetPid();
        stopMotors();

        Serial.println("MOTORS DISABLED");
    }
    else
    {
        // Increment the integral part of the PID loop
        if (throttle > PID_THROTTLE_THRESHOLD)
        {
            runPidOptimizer();
            calculatePidIntegral(gyro.roll(), gyro.pitch(), gyro.yaw());
        }
        else
        {
            resetPid();
        }

        // To debug stuff
        // print();
        // printConstants();
        // printThrottle();
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(gyro.roll(), gyro.pitch(), gyro.yaw());

        savePidErrors(gyro.roll(), gyro.pitch(), gyro.yaw());
    
        persistPidConstants();

        delayToKeepFeedbackLoopHz(start_micros_timestamp);
    }
}

void KlevebrandMaxJetDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    // Not implemented
}
