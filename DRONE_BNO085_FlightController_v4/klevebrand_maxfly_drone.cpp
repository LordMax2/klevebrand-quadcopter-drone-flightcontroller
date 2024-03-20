#include "klevebrand_maxfly_drone.h"

void Reciver::setup() {
  Serial.println("Setting up Radio...");

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pAddress);
  radio.openWritingPipe(fAddress);

  radio.printDetails();

  //radio.printDetails();
  radio.startListening();

  delay(5000);
  Serial.println("Radio is set up!");
}

void Reciver::recive() {
  float recivedDataArray[8];

  if(radio.available()) {
    radio.read(&recivedDataArray, sizeof(recivedDataArray));

    reciverData.lastRecivedMessageMillis = millis();

    reciverData.inputThrottle = recivedDataArray[0];
    reciverData.yawDesiredAngle = recivedDataArray[1];
    reciverData.rollDesiredAngle = recivedDataArray[2];
    reciverData.pitchDesiredAngle = recivedDataArray[3];

    if (recivedDataArray[4] == 1.0f) {
        reciverData.flightMode = acro;
    } else if (recivedDataArray[4] == 2.0f) {
        reciverData.flightMode = autoLevel;
    } else if (recivedDataArray[4] == 3.0f) {
        reciverData.flightMode = holdPosition;
    } else {
        reciverData.flightMode = autoLevel; 
    }
  }
}

void Reciver::transmit(float roll, float pitch, float yaw, FlightMode flightMode) {  
  float feedbackArray[5];

  feedbackArray[0] = roll;
  feedbackArray[1] = pitch;
  feedbackArray[2] = yaw;

  switch (flightMode) {
    case acro:
      feedbackArray[3] = 1.0f;
    case autoLevel:
      feedbackArray[3] = 2.0f;
    case holdPosition:
      feedbackArray[3] = 3.0f;
    default:
      feedbackArray[3] = 0.0f;
  }

  radio.stopListening();
  radio.write( &feedbackArray, sizeof(feedbackArray));
  radio.startListening();
}

void Gyro::setReports() {
  if (!bno08x.enableReport(reportType, reportIntervalUs)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void Gyro::setup() {
  Serial.println("Setting up gyroscope");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  Serial.println("BNO085 set up!");
  
  setReports();

  delay(5000);
}

void Gyro::update() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
  }
}

void Gyro::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void Gyro::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void Gyro::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void Drone::setup() {
  Serial.println("Starting drone...");

  gyro.setup();
  reciver.setup();
  setupMotors();
  
  Serial.println("Drone started!");

  // Fake reciver response for testing
  //reciver.reciverData.flightMode = 1;
  //reciver.reciverData.inputThrottle = 1200;
  //reciver.reciverData.yawDesiredAngle = 0;
  //reciver.reciverData.pitchDesiredAngle = 0;
  //reciver.reciverData.rollDesiredAngle = 0;
}

void Drone::setupMotors() {
  Serial.println("Setting up motors...");

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  motorLF.attach(5);
  motorRF.attach(6);
  motorLB.attach(7);
  motorRB.attach(8);

  motorLF.writeMicroseconds(1000);
  motorRF.writeMicroseconds(1000);
  motorLB.writeMicroseconds(1000);
  motorRB.writeMicroseconds(1000);
  
  Serial.println("Motors setup!");
}

void Drone::regulateInputsPID() {
  // Constrain yawDesiredAngle within the range [0, 360]
  reciver.reciverData.yawDesiredAngle = constrain(reciver.reciverData.yawDesiredAngle, 0, 360);

  // Constrain inputThrottle within the range [1000, 1600]
  reciver.reciverData.inputThrottle = constrain(reciver.reciverData.inputThrottle, 1000, 1600);

  if (millis() >= 15000) launchMode = false;
}

void Drone::setZero() {
  if(setZeroBool) {
    pid.pitchOffset = Drone::gyro.ypr.pitch;
    pid.rollOffset = Drone::gyro.ypr.roll;
    setZeroBool = false;
  }
}

void Drone::stopMotors() {
    motorLF.writeMicroseconds(1000);
    motorRF.writeMicroseconds(1000);
    motorLB.writeMicroseconds(1000);
    motorRB.writeMicroseconds(1000);
}

void Drone::runMotors() {
    motorLF.writeMicroseconds(pid.pid_throttle_L_F);
    motorRF.writeMicroseconds(pid.pid_throttle_R_F);
    motorLB.writeMicroseconds(pid.pid_throttle_L_B);
    motorRB.writeMicroseconds(pid.pid_throttle_R_B);
}

void Drone::calculatePID() {
  pid.calculate(Drone::reciver.reciverData.inputThrottle, launchMode, gyro.ypr.roll, gyro.ypr.pitch, gyro.ypr.yaw);
}

bool Drone::lostConnection() {
  return millis() - reciver.reciverData.lastRecivedMessageMillis >= 400;
}

void Drone::updateGyro() {
  gyro.update();
}

euler_t Drone::getGyroYpr() {
  return gyro.ypr;
}

void Drone::reciverRecive() {
  reciver.recive();
}

void Drone::resetPID() {
  pid.reset();
}

void Drone::regulateThrottlePID() {
  pid.regulateThrottle();
}

void Drone::printThrottle() {
  Serial.print(pid.pid_throttle_L_F);
  Serial.print("    ");
  Serial.println(pid.pid_throttle_R_F);
  Serial.print(pid.pid_throttle_L_B);
  Serial.print("    ");
  Serial.println(pid.pid_throttle_R_B);
  Serial.println("-----------------------------------------");
}

void PID::regulateThrottle() {
  pid_throttle_R_F = constrain(pid_throttle_R_F, 1100, 2000);
  pid_throttle_L_F = constrain(pid_throttle_L_F, 1100, 2000);
  pid_throttle_R_B = constrain(pid_throttle_R_B, 1100, 2000);
  pid_throttle_L_B = constrain(pid_throttle_L_B, 1100, 2000);
}


void PID::reset() {
  pid_throttle_L_F = 1000;
  pid_throttle_L_B = 1000;
  pid_throttle_R_F = 1000;
  pid_throttle_R_B = 1000;

  pitch_PID = 0, roll_PID = 0, yaw_PID = 0;
  roll_error = 0, pitch_error = 0, yaw_error = 0;
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;

  yaw_pid_p = 0, roll_pid_p = 0, pitch_pid_p = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
  yaw_pid_d = 0, roll_pid_d = 0, pitch_pid_d = 0;
}

void PID::calculate(float throttle, bool launchMode, float x, float y, float z) {
    if (throttle <= PID_THROTTLE_THRESHOLD) {
        reset();
        return;
    }

    roll_error = roll_desired_angle - x;
    pitch_error = pitch_desired_angle - y;
    yaw_error = yaw_desired_angle - z;

    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;
    yaw_pid_p = yaw_kp * yaw_error;

    unsigned long currentTimer = millis();
    if (currentTimer - previousTimer >= PID_UPDATE_INTERVAL) {
        previousTimer = currentTimer;

        if (launchMode) {
            resetIntegral();
        } else {
            updateIntegral();
        }

        roll_pid_d = roll_kd * (roll_error - roll_previous_error);
        pitch_pid_d = pitch_kd * (pitch_error - pitch_previous_error);
        yaw_pid_d = yaw_kd * (yaw_error - yaw_previous_error);

        roll_previous_error = roll_error;
        pitch_previous_error = pitch_error;
        yaw_previous_error = yaw_error;

        roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
        pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
        yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;

        constrainPID(roll_PID);
        constrainPID(pitch_PID);
        constrainYawPID(yaw_PID);

        pid_throttle_L_F = throttle + roll_PID - pitch_PID - yaw_PID;
        pid_throttle_R_F = throttle - roll_PID - pitch_PID + yaw_PID;
        pid_throttle_L_B = throttle + roll_PID + pitch_PID + yaw_PID;
        pid_throttle_R_B = throttle - roll_PID + pitch_PID - yaw_PID;

        regulateThrottle();
    }
}

void PID::updateIntegral() {
    roll_pid_i += roll_ki * roll_error;
    pitch_pid_i += pitch_ki * pitch_error;
    yaw_pid_i += yaw_ki * yaw_error;

    roll_pid_i = constrain(roll_pid_i, -pid_max, pid_max);
    pitch_pid_i = constrain(pitch_pid_i, -pid_max, pid_max);
    yaw_pid_i = constrain(yaw_pid_i, -yaw_pid_max, yaw_pid_max);
}

void PID::resetIntegral() {
    roll_pid_i = 0;
    pitch_pid_i = 0;
    yaw_pid_i = 0;
}

void PID::constrainPID(float &pid_value) {
    if (pid_value > pid_max) {
        pid_value = pid_max;
    } else if (pid_value < -pid_max) {
        pid_value = -pid_max;
    }
}

void PID::constrainYawPID(float &yaw_PID) {
    if (yaw_PID > yaw_pid_max) {
        yaw_PID = yaw_pid_max;
    } else if (yaw_PID < -yaw_pid_max) {
        yaw_PID = -yaw_pid_max;
    }
}

