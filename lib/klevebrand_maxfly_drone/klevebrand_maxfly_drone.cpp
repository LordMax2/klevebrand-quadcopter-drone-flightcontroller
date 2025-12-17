#include "klevebrand_maxfly_drone.h"

void KlevebrandMaxFlyDrone::setup()
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

void KlevebrandMaxFlyDrone::run()
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

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    motorLeftFront().writeMicroseconds(pid.pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motorRightFront().writeMicroseconds(pid.pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motorLeftBack().writeMicroseconds(pid.pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motorRightBack().writeMicroseconds(pid.pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
}

void KlevebrandMaxFlyDrone::printThrottle()
{
    Serial.print(pid.pidThrottleLF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print(pid.pidThrottleLB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.println("-----------------------------------------");
}
