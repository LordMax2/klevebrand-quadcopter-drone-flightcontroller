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

void KlevebrandMaxFlyDrone::persistPidConstants()
{
    if(millis() - last_pid_persist_timestamp_milliseconds >= PID_PERSIST_INTERVAL_MILLISECONDS)
    {
        int address = 128;

        switch (getFlightMode())
        {
        case acro:
            address = 128;
            break;
        case auto_level:
            address = 256;
            break;
        }

        PidConstants pid_constants = PidConstants(
            pid.getYawKp(), pid.getYawKi(), pid.getYawKd(),
            pid.getPitchKp(), pid.getPitchKi(), pid.getPitchKd(),
            pid.getRollKp(), pid.getRollKi(), pid.getRollKd());

        eeprom_pid_repository.save(pid_constants, address);

        last_pid_persist_timestamp_milliseconds = millis();
    }
}

void KlevebrandMaxFlyDrone::runPidOptimizer()
{
    pid.runRollOptimizer(gyro.roll(), roll_desired_angle);
    pid.runPitchOptimizer(gyro.pitch(), pitch_desired_angle);
    pid.runYawOptimizer(gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
}

void KlevebrandMaxFlyDrone::delayToKeepFeedbackLoopHz(long start_micros_timestamp)
{
    long current_micros_timestamp = micros();

    long microseconds_feedback_loop_should_take = 1000000 / FEEDBACK_LOOP_HZ;

    long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take)
    {
        delayMicroseconds(microseconds_left_for_loop);
    }
}

void KlevebrandMaxFlyDrone::setupMotors()
{
    Serial.println("SETTING UP MOTORS...");

    pinMode(motor_left_front_pin_number, OUTPUT);
    pinMode(motor_right_front_pin_number, OUTPUT);
    pinMode(motor_left_back_pin_number, OUTPUT);
    pinMode(motor_right_back_pin_number, OUTPUT);

    motor_left_front.attach(motor_left_front_pin_number);
    motor_right_front.attach(motor_right_front_pin_number);
    motor_left_back.attach(motor_left_back_pin_number);
    motor_right_back.attach(motor_right_back_pin_number);

    motor_left_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_left_back.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_back.writeMicroseconds(THROTTLE_MINIMUM);

    delay(1000);

    Serial.println("MOTORS SETUP!");
}

void KlevebrandMaxFlyDrone::stopMotors()
{
    motor_left_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_left_back.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_back.writeMicroseconds(THROTTLE_MINIMUM);
}

void KlevebrandMaxFlyDrone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    motor_left_front.writeMicroseconds(pid.pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_right_front.writeMicroseconds(pid.pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_left_back.writeMicroseconds(pid.pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_right_back.writeMicroseconds(pid.pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
}

void KlevebrandMaxFlyDrone::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode);
}

bool KlevebrandMaxFlyDrone::hasLostConnection()
{
    bool transmitter_lost_connection = millis() - throttle_set_timestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    bool gyro_lost_connection = millis() - gyro.timestamp_milliseconds() >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    return transmitter_lost_connection || gyro_lost_connection;
}

bool KlevebrandMaxFlyDrone::updateGyro()
{
    return gyro.reload();
}

void KlevebrandMaxFlyDrone::printGyro()
{
    gyro.printYawPitchRoll();
}

void KlevebrandMaxFlyDrone::resetPid()
{
    pid.reset();
}

void KlevebrandMaxFlyDrone::setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd)
{
    pid = QuadcopterPid(yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd);
}

void KlevebrandMaxFlyDrone::disableMotors()
{
    is_motors_enabled = false;
}

void KlevebrandMaxFlyDrone::enableMotors()
{
    is_motors_enabled = true;
}

bool KlevebrandMaxFlyDrone::isMotorsEnabled()
{
    return is_motors_enabled;
}

void KlevebrandMaxFlyDrone::setThrottle(float value)
{
    throttle = value;
    throttle_set_timestamp = millis();
}

void KlevebrandMaxFlyDrone::setDesiredYawAngle(float value)
{
    yaw_desired_angle = value;
    yaw_desired_angle_set_timestamp = millis();
}

void KlevebrandMaxFlyDrone::setDesiredPitchAngle(float value)
{
    pitch_desired_angle = value;
    desired_pitch_angle_set_timestamp = millis();
}

void KlevebrandMaxFlyDrone::setDesiredRollAngle(float value)
{
    roll_desired_angle = value;
    desired_roll_angle_set_timestamp = millis();
}

void KlevebrandMaxFlyDrone::savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.savePitchError(gyro_pitch, pitch_desired_angle);
    pid.saveRollError(gyro_roll, roll_desired_angle);
    pid.saveYawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode);
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

void KlevebrandMaxFlyDrone::setYawCompassMode(bool yaw_compass_mode)
{
    this->yaw_compass_mode = yaw_compass_mode;
}

void KlevebrandMaxFlyDrone::printPid()
{
    pid.printPid(gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
}

void KlevebrandMaxFlyDrone::printPidConstants()
{
    pid.printConstants();
}

void KlevebrandMaxFlyDrone::setFlightMode(FlightMode flight_mode)
{
    KlevebrandMaxFlyDrone::flight_mode = flight_mode;
}

void KlevebrandMaxFlyDrone::setFlightModeAutoLevel()
{
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level)
        return;

    setFlightMode(auto_level);

    gyro.reset();

    delay(1000);

    gyro.setReportModeEuler();

    PidConstants pid_constants = eeprom_pid_repository.get(256);

    if (pid_constants.isValid())
    {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

        pid_constants.print();
    }
    else
    {
        setPidConstants(1.25, 0.01, 25, 1.25, 0.01, 25, 0.5, 0.005, 2);
    }

    setYawCompassMode(true);

    Serial.println("FLIGHT MODE AUTOLEVEL");
}

void KlevebrandMaxFlyDrone::setFlightModeAcro()
{
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == acro)
        return;

    setFlightMode(acro);

    gyro.reset();

    delay(1000);

    gyro.setReportModeAcro();

    PidConstants pid_constants = eeprom_pid_repository.get(128);

    if (pid_constants.isValid())
    {
        setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd,
                        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

        pid_constants.print();
    }
    else
    {
        setPidConstants(0.4, 0.02, 6, 0.4, 0.02, 6, 0.4, 0.02, 6);
    }

    setYawCompassMode(false);

    Serial.println("FLIGHT MODE ACRO");
}

FlightMode KlevebrandMaxFlyDrone::getFlightMode()
{
    return flight_mode;
}