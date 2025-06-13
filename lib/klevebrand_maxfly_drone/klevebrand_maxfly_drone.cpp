#include "klevebrand_maxfly_drone.h"

void Drone::setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  if (!Serial)
  {
    Serial.println("Failed to start serial...");
  }
  while (!Serial)
    ;

  Serial.println("Starting drone...");

  gyro.setup();
  setupMotors();

  Serial.println("Drone started!");
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
    // If connection is good, check if we should set the PID zero offset
    setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();

    // Then calculate the PID stabilization
    calculatePid(gyro.roll(), gyro.pitch(), gyro.yaw());

    // To debug throttle response
    // printPid();
    // printThrottle();
    // printGyro();

    // Run the motors with the calculated PID throttle
    runMotors(gyro.roll(), gyro.pitch(), gyro.yaw());

    savePidErrors(gyro.roll(), gyro.pitch(), gyro.yaw());

    delayToKeepFeedbackLoopHz(start_micros_timestamp);
  }
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
  Serial.println("Setting up motors...");

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

  Serial.println("Motors setup!");
}

void Drone::setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero()
{
  if (set_zero_bool)
  {
    pid.setPitchOffset(gyro.pitch());
    pid.setRollOffset(gyro.roll());
    set_zero_bool = false;
  }
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
  motor_left_front.writeMicroseconds(pid.pidThrottleLF(throttle, gyro_roll, gyro_pitch, gyro_yaw));
  motor_right_front.writeMicroseconds(pid.pidThrottleRF(throttle, gyro_roll, gyro_pitch, gyro_yaw));
  motor_left_back.writeMicroseconds(pid.pidThrottleLB(throttle, gyro_roll, gyro_pitch, gyro_yaw));
  motor_right_back.writeMicroseconds(pid.pidThrottleRB(throttle, gyro_roll, gyro_pitch, gyro_yaw));
}

void Drone::calculatePid(float gyro_roll, float gyro_pitch, float gyro_yaw)
{
  pid.updateIntegral(gyro_roll, gyro_pitch, gyro_yaw);
}

bool Drone::hasLostConnection()
{
  return millis() - throttle_set_timestamp >= TRANSMITION_TIMEOUT_DEFINITION_MILLISECONDS;
}

void Drone::updateGyro()
{
  gyro.reload();
}

void Drone::printGyro()
{
  gyro.printYawPitchRoll();
}

void Drone::resetPid()
{
  pid.reset();
}

void Drone::setThrottleYawPitchRollFromReceiver(PwmReceiver receiver)
{
  setThrottle(receiver.getChannelValue(throttle_receiver_channel_number));

  // Currently disabled unitl successful hover flight
  // setDesiredYawAngle(receiver.getChannelValue(yawReceiverChannelNumber));
  // setDesiredPitchAngle(receiver.getChannelValue(pitchReceiverChannelNumber));
  // setDesiredRollAngle(receiver.getChannelValue(rollReceiverChannelNumber));
}

void Drone::setPidFromReceiver(PwmReceiver receiver)
{
  setPidPConstant(receiver.getChannelValue(pid_p_constant_channel_number));
  setPidIConstant(receiver.getChannelValue(pid_i_constant_channel_number));
  setPidDConstant(receiver.getChannelValue(pid_d_constant_channel_number));
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Drone::setPidPConstant(float pwm_value)
{
  pid.roll_kp = mapfloat(pwm_value, 1100, 1800, 0, 10);
  pid.pitch_kp = mapfloat(pwm_value, 1100, 1800, 0, 10);

  if (pid.roll_kp < 0)
    pid.roll_kp = 0;
  if (pid.pitch_kp < 0)
    pid.pitch_kp = 0;
}

void Drone::setPidIConstant(float pwm_value)
{
  pid.roll_ki = mapfloat(pwm_value, 1100, 1800, 0, 5);
  pid.pitch_ki = mapfloat(pwm_value, 1100, 1800, 0, 5);

  if (pid.roll_ki < 0)
    pid.roll_ki = 0;
  if (pid.pitch_ki < 0)
    pid.pitch_ki = 0;
}

void Drone::setPidDConstant(float pwm_value)
{
  pid.roll_kd = mapfloat(pwm_value, 1100, 1800, 0, 40);
  pid.pitch_kd = mapfloat(pwm_value, 1100, 1800, 0, 40);

  if (pid.roll_kd < 0)
    pid.roll_kd = 0;
  if (pid.pitch_kd < 0)
    pid.pitch_kd = 0;
}

void Drone::setThrottle(float value)
{
  if (value > 1800)
  {
    value = 2000;
  }

  throttle = value;
  throttle_set_timestamp = millis();
}

void Drone::setDesiredYawAngle(float value)
{
  desired_yaw_angle = value;
  desired_yaw_angle_set_timestamp = millis();
}

void Drone::setDesiredPitchAngle(float value)
{
  desired_pitch_angle = value;
  desired_pitch_angle_set_timestamp = millis();
}

void Drone::setDesiredRollAngle(float value)
{
  desired_roll_angle = value;
  desired_roll_angle_set_timestamp = millis();
}

void Drone::printThrottle()
{
  Serial.print(pid.pidThrottleLF(throttle, gyro.roll(), gyro.pitch(), gyro.yaw()));
  Serial.print("    ");
  Serial.println(pid.pidThrottleRF(throttle, gyro.roll(), gyro.pitch(), gyro.yaw()));
  Serial.print(pid.pidThrottleLB(throttle, gyro.roll(), gyro.pitch(), gyro.yaw()));
  Serial.print("    ");
  Serial.println(pid.pidThrottleRB(throttle, gyro.roll(), gyro.pitch(), gyro.yaw()));
  Serial.println("-----------------------------------------");
}