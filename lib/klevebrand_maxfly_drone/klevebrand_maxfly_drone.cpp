#include "klevebrand_maxfly_drone.h"
#include "components/flight_mode/flight_mode.h"

void Drone::setup()
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

    setupMotors();

    setFlightModeAcro();

    Serial.println("DRONE STARTED!");
}

void Drone::run()
{
    long start_micros_timestamp = micros();

    // Get the latest data from the gyroscope
    updateGyro();

    // Check if connection is alive
    if (hasLostConnection())
    {
        // If connection is dead, stop the drone
        resetPid();
        stopMotors();

        Serial.println("LOST CONNECTION");
    }
    else 
    {
        // Increment the integral part of the PID loop
        if (throttle > PID_THROTTLE_THRESHOLD)
        {
            calculatePidIntegral(gyro.roll(), gyro.pitch(), gyro.yaw());
        }
        else
        {
            resetPid();
        }

        // To debug throttle response
        // printPid();
        // printPidConstants();
        // printThrottle();
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(gyro.roll(), gyro.pitch(), gyro.yaw());

        savePidErrors(gyro.roll(), gyro.pitch(), gyro.yaw());

        saveMeasurements();

        delayToKeepFeedbackLoopHz(start_micros_timestamp);
    }
}

void Drone::saveMeasurements()
{
    pid.saveMeasurements(
        gyro.pitch(), 
        pitch_desired_angle, 
        gyro.roll(), 
        roll_desired_angle, 
        micros()
    );
}

void Drone::delayToKeepFeedbackLoopHz(long start_micros_timestamp)
{
    long current_micros_timestamp = micros();

    long microseconds_feedback_loop_should_take = 1000000 / FEEDBACK_LOOP_HZ;

    long microseconds_left_for_loop = microseconds_feedback_loop_should_take - (current_micros_timestamp - start_micros_timestamp);

    if (microseconds_left_for_loop > 0 && microseconds_left_for_loop < microseconds_feedback_loop_should_take)
    {
        delayMicroseconds(microseconds_left_for_loop);
    }

    // Serial.println(current_micros_timestamp - start_micros_timestamp);
}

void Drone::setupMotors()
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

void Drone::stopMotors()
{
    motor_left_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_front.writeMicroseconds(THROTTLE_MINIMUM);
    motor_left_back.writeMicroseconds(THROTTLE_MINIMUM);
    motor_right_back.writeMicroseconds(THROTTLE_MINIMUM);
}

void Drone::runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    motor_left_front.writeMicroseconds(pid.pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motor_right_front.writeMicroseconds(pid.pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motor_left_back.writeMicroseconds(pid.pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
    motor_right_back.writeMicroseconds(pid.pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle));
}

void Drone::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle);
}

bool Drone::hasLostConnection()
{
    bool transmitter_lost_connection = millis() - throttle_set_timestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    bool gyro_lost_connection = millis() - gyro.timestamp_milliseconds() >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;

    return transmitter_lost_connection || gyro_lost_connection;
}

bool Drone::updateGyro()
{
    return gyro.reload();
}

void Drone::printGyro()
{
    gyro.printYawPitchRoll();
}

void Drone::resetPid()
{
    pid.reset();
}

void Drone::setPidPConstant(float value)
{
    pid.roll_kp = value;
    pid.pitch_kp = value;

    if (pid.roll_kp < 0)
        pid.roll_kp = 0;
    if (pid.pitch_kp < 0)
        pid.pitch_kp = 0;
}

void Drone::setPidIConstant(float value)
{
    pid.roll_ki = value;
    pid.pitch_ki = value;

    if (pid.roll_ki < 0)
        pid.roll_ki = 0;
    if (pid.pitch_ki < 0)
        pid.pitch_ki = 0;
}

void Drone::setPidDConstant(float value)
{
    pid.roll_kd = value;
    pid.pitch_kd = value;

    if (pid.roll_kd < 0)
        pid.roll_kd = 0;
    if (pid.pitch_kd < 0)
        pid.pitch_kd = 0;
}

void Drone::setThrottle(float value)
{
    throttle = value;
    throttle_set_timestamp = millis();
}

void Drone::setDesiredYawAngle(float value)
{
    yaw_desired_angle = value;
    yaw_desired_angle_set_timestamp = millis();
}

void Drone::setDesiredPitchAngle(float value)
{
    pitch_desired_angle = value;
    desired_pitch_angle_set_timestamp = millis();
}

void Drone::setDesiredRollAngle(float value)
{
    roll_desired_angle = value;
    desired_roll_angle_set_timestamp = millis();
}

void Drone::savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.savePitchError(gyro_pitch, pitch_desired_angle);
    pid.saveRollError(gyro_roll, roll_desired_angle);
    pid.saveYawError(gyro_yaw, yaw_desired_angle);
}

void Drone::printThrottle()
{
    Serial.print(pid.pidThrottleLF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle));
    Serial.print(pid.pidThrottleLB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle));
    Serial.println("-----------------------------------------");
}

void Drone::printPid()
{
    pid.printPid(gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle);
}

void Drone::printPidConstants()
{
    pid.printPidConstants();
}

void Drone::setFlightMode(FlightMode flight_mode)
{
    Drone::flight_mode = flight_mode;
}

void Drone::setFlightModeAutoLevel()
{
    if (flight_mode != auto_level)
    {
        gyro.setReportModeEuler();
    }

    setPidPConstant(1.25);
    setPidIConstant(0.01);
    setPidDConstant(25);

    setFlightMode(auto_level);
}

void Drone::setFlightModeAcro()
{
    if (flight_mode != acro)
    {
        gyro.setReportModeAcro();
    }

    setPidPConstant(6);
    setPidIConstant(0);
    setPidDConstant(14);

    setFlightMode(acro);
}

FlightMode Drone::getFlightMode()
{
    return flight_mode;
}