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

    Wire.begin();

    eeprom.begin();
    if (!eeprom.isConnected())
    {
        Serial.println("ERROR: CAN'T FIND EEPROMD...");
    }

    Serial.print("EEPROM CONNECTION STATUS:\t");
    Serial.println(eeprom.isConnected());

    setupMotors();

    setFlightModeAcro();

    Serial.println("DRONE STARTED!");
}

void Drone::run()
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
        // printPid();
        // printPidConstants();
        // printThrottle();
        // printGyro();

        // Run the motors with the calculated PID throttle
        runMotors(gyro.roll(), gyro.pitch(), gyro.yaw());

        savePidErrors(gyro.roll(), gyro.pitch(), gyro.yaw());

        delayToKeepFeedbackLoopHz(start_micros_timestamp);
    }
}

void Drone::persistPidConstants() 
{
    int address = 256;

    float yawKp = pid.getYawKp();
    eeprom.setBlock(address, (uint8_t*) &yawKp, sizeof(yawKp));
    
    address += sizeof(yawKp) * 2;

    float yawKi = pid.getYawKi();
    eeprom.setBlock(address, (uint8_t*) &yawKi, sizeof(yawKi));
    
    address += sizeof(yawKi) * 2;

    float yawKd = pid.getYawKd();
    eeprom.setBlock(address, (uint8_t*) &yawKd, sizeof(yawKd));

    address += sizeof(yawKd) * 2;

    float pitchKp = pid.getPitchKp();
    eeprom.setBlock(address, (uint8_t*) &pitchKp, sizeof(pitchKp));
    
    address += sizeof(pitchKp) * 2;

    float pitchKi = pid.getPitchKi();
    eeprom.setBlock(address, (uint8_t*) &pitchKi, sizeof(pitchKi));
    
    address += sizeof(pitchKi) * 2;

    float pitchKd = pid.getPitchKd();
    eeprom.setBlock(address, (uint8_t*) &pitchKd, sizeof(pitchKd));

    address += sizeof(pitchKd) * 2;

    float rollKp = pid.getRollKp();
    eeprom.setBlock(address, (uint8_t*) &rollKp, sizeof(rollKp));
    
    address += sizeof(rollKp) * 2;

    float rollKi = pid.getPitchKi();
    eeprom.setBlock(address, (uint8_t*) &rollKi, sizeof(rollKi));
    
    address += sizeof(rollKi) * 2;

    float rollKd = pid.getPitchKd();
    eeprom.setBlock(address, (uint8_t*) &rollKd, sizeof(rollKd));
}

void Drone::runPidOptimizer()
{
    pid.runRollOptimizer(gyro.roll(), roll_desired_angle);
    pid.runPitchOptimizer(gyro.pitch(), pitch_desired_angle);
    pid.runYawOptimizer(gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
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
    motor_left_front.writeMicroseconds(pid.pidThrottleLF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_right_front.writeMicroseconds(pid.pidThrottleRF(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_left_back.writeMicroseconds(pid.pidThrottleLB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
    motor_right_back.writeMicroseconds(pid.pidThrottleRB(throttle, gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode));
}

void Drone::calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.updateIntegral(gyro_roll, roll_desired_angle, gyro_pitch, pitch_desired_angle, gyro_yaw, yaw_desired_angle, yaw_compass_mode);
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

void Drone::setPidConstants(float kp, float ki, float kd)
{
    pid = QuadcopterPid(kp, ki, kd);
}

void Drone::setPidConstants(float kp, float ki, float kd, float yaw_kp, float yaw_ki, float yaw_kd)
{
    pid = QuadcopterPid(kp, ki, kd, yaw_kp, yaw_ki, yaw_kd);
}

void Drone::disableMotors()
{
    is_motors_enabled = false;
}

void Drone::enableMotors()
{
    is_motors_enabled = true;
}

bool Drone::isMotorsEnabled()
{
    return is_motors_enabled;
}

void Drone::setThrottle(float value)
{
    if (value > 1800)
    {
        value = 1800;
    }

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
    pid.saveYawError(gyro_yaw, yaw_desired_angle, yaw_compass_mode);
}

void Drone::printThrottle()
{
    Serial.print(pid.pidThrottleLF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRF(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print(pid.pidThrottleLB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.print("    ");
    Serial.println(pid.pidThrottleRB(throttle, gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode));
    Serial.println("-----------------------------------------");
}

void Drone::setYawCompassMode(bool yaw_compass_mode)
{
    this->yaw_compass_mode = yaw_compass_mode;
}

void Drone::printPid()
{
    pid.printPid(gyro.roll(), roll_desired_angle, gyro.pitch(), pitch_desired_angle, gyro.yaw(), yaw_desired_angle, yaw_compass_mode);
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
    // Temprorary return early util I have connected the IMU's reset pin
    if (getFlightMode() == auto_level) return;

    gyro.reset();

    delay(1000);

    gyro.setReportModeEuler();

    setPidConstants(1.25, 0.01, 25, 0.5, 0.005, 2);

    setFlightMode(auto_level);

    setYawCompassMode(true);
    
    Serial.println("FLIGHT MODE AUTOLEVEL");
}

void Drone::setFlightModeAcro()
{
    // Temprorary return early util I have connected the IMU's reset pin
    if(getFlightMode() == acro) return;

    gyro.reset();

    delay(1000);

    gyro.setReportModeAcro();

    setPidConstants(0.4, 0.02, 6);

    setFlightMode(acro);

    setYawCompassMode(false);

    Serial.println("FLIGHT MODE ACRO");
}

FlightMode Drone::getFlightMode()
{
    return flight_mode;
}