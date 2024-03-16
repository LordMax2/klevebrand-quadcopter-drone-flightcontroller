#include "trest_drone.h"

void Reciver::setup() {
  Serial.println("Setting up Radio...");

  if (!Reciver::radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}
  }
  Reciver::radio.setPALevel(RF24_PA_MAX);
  Reciver::radio.openReadingPipe(1, pAddress);
  Reciver::radio.openWritingPipe(fAddress);

  Reciver::radio.printDetails();

  //radio.printDetails();
  Reciver::radio.startListening();

  delay(5000);
  Serial.println("Radio is set up!");
}

void Reciver::recive() {
  float recivedDataArray[8];

  if(radio.available()) {
    radio.read(&recivedDataArray, sizeof(recivedDataArray));

    Reciver::reciverData.lastRecivedMessageMillis = millis();

    Reciver::reciverData.inputThrottle = recivedDataArray[0];
    Reciver::reciverData.yawDesiredAngle = recivedDataArray[1];
    Reciver::reciverData.rollDesiredAngle = recivedDataArray[2];
    Reciver::reciverData.pitchDesiredAngle = recivedDataArray[3];

    if (recivedDataArray[4] == 1.0f) {
        Reciver::reciverData.flightMode = acro;
    } else if (recivedDataArray[4] == 2.0f) {
        Reciver::reciverData.flightMode = autoLevel;
    } else if (recivedDataArray[4] == 3.0f) {
        Reciver::reciverData.flightMode = holdPosition;
    } else {
        Reciver::reciverData.flightMode = autoLevel; 
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

  Reciver::radio.stopListening();
  Reciver::radio.write( &feedbackArray, sizeof(feedbackArray));
  Reciver::radio.startListening();
}

void Gyro::setReports() {
  if (!Gyro::bno08x.enableReport(Gyro::reportType, Gyro::reportIntervalUs)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void Gyro::setup() {
  Serial.println("Setting up gyroscope");
  if (!Gyro::bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  Serial.println("BNO085 set up!");
  
  Gyro::setReports();

  delay(5000);
}

void Gyro::get() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        Gyro::quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        Gyro::quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
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

  Drone::gyro.setup();
  Drone::reciver.setup();
  Drone::setupMotors();
  
  Serial.println("Drone started!");
}

void Drone::setupMotors() {
  Serial.println("Setting up motors...");

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  Drone::motorLF.attach(5);
  Drone::motorRF.attach(6);
  Drone::motorLB.attach(7);
  Drone::motorRB.attach(8);

  Drone::motorLF.writeMicroseconds(1000);
  Drone::motorRF.writeMicroseconds(1000);
  Drone::motorLB.writeMicroseconds(1000);
  Drone::motorRB.writeMicroseconds(1000);
  
  Serial.println("Motors setup!");
}

void Drone::regulateInputsPID() {
  if (Drone::reciver.reciverData.yawDesiredAngle > 360) {
    Drone::reciver.reciverData.yawDesiredAngle = 0;
  } else if (Drone::reciver.reciverData.yawDesiredAngle < 0) {
    Drone::reciver.reciverData.yawDesiredAngle = 360;
  }

  if (Drone::reciver.reciverData.inputThrottle > 1600) {
    Drone::reciver.reciverData.inputThrottle = 1600;
  } else if (Drone::reciver.reciverData.inputThrottle < 1000) {
    Drone::reciver.reciverData.inputThrottle = 1000;
  }

  if (millis() >= 15000) {
    Drone::launchMode = false;
  }
}

void Drone::setZero() {
  if(Drone::setZeroBool) {
    Drone::pid.pitchOffset = Drone::gyro.ypr.pitch;
    Drone::pid.rollOffset = Drone::gyro.ypr.roll;
    Drone::setZeroBool = false;
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
  Drone::pid.calculate(Drone::reciver.reciverData.inputThrottle, Drone::launchMode, Drone::gyro.ypr.roll, Drone::gyro.ypr.pitch, Drone::gyro.ypr.yaw);
}

bool Drone::lostConnection() {
  return millis() - Drone::reciver.reciverData.lastRecivedMessageMillis >= 400;
}

void PID::regulateThrottle() {
  /* Regulate throttle for ESCs */
    //Right front
    if (pid_throttle_R_F < 1100) {
      pid_throttle_R_F = 1100;
    }
    if (pid_throttle_R_F > 2000) {
      pid_throttle_R_F = 2000;
    }

    //Left front
    if (pid_throttle_L_F < 1100) {
      pid_throttle_L_F = 1100;
    }
    if (pid_throttle_L_F > 2000) {
      pid_throttle_L_F = 2000;
    }

    //Right back
    if (pid_throttle_R_B < 1100) {
      pid_throttle_R_B = 1100;
    }
    if (pid_throttle_R_B > 2000) {
      pid_throttle_R_B = 2000;
    }

    //Left back
    if (pid_throttle_L_B < 1100) {
      pid_throttle_L_B = 1100;
    }
    if (pid_throttle_L_B > 2000) {
      pid_throttle_L_B = 2000;
    }
}

void PID::reset() {
  PID::pid_throttle_L_F = 1000;
  PID::pid_throttle_L_B = 1000;
  PID::pid_throttle_R_F = 1000;
  PID::pid_throttle_R_B = 1000;

  PID::pitch_PID = 0, PID::roll_PID = 0, PID::yaw_PID = 0;
  PID::roll_error = 0, PID::pitch_error = 0, PID::yaw_error = 0;
  PID::roll_previous_error = 0, PID::pitch_previous_error = 0, PID::yaw_previous_error = 0;

  PID::yaw_pid_p = 0, PID::roll_pid_p = 0, PID::pitch_pid_p = 0;
  PID::yaw_pid_i = 0, PID::roll_pid_i = 0, PID::pitch_pid_i = 0;
  PID::yaw_pid_d = 0, PID::roll_pid_d = 0, PID::pitch_pid_d = 0;
}

void PID::calculate(float throttle, bool launchMode, float x, float y, float z) {
  if (throttle > 1050) {
    /* PID */
    roll_error = roll_desired_angle - y;
    pitch_error = pitch_desired_angle - z;
    yaw_error = yaw_desired_angle - x;

    //Proportional dowsn't need any delay any change can be used instantly
    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;
    yaw_pid_p = yaw_kp * yaw_error;

    //Integral and Derivative need a delay 10 ms should be enough. We will use an exact delay to speed up the process
    // if you dont use a delay integral will wind up fast
    // Derivitive will not see any substantial change to be usefull

    static unsigned long _ExactTimer;
    if (( millis() - _ExactTimer) >= (10)) {
      _ExactTimer += (10);
      if (( millis() - _ExactTimer) >= (1))_ExactTimer = millis(); // prevents timer windup
      // Integral
      if (launchMode) {
        roll_pid_i = 0;
        pitch_pid_i = 0;
        yaw_pid_i = 0;
      } else {
        roll_pid_i = roll_pid_i + (roll_ki * roll_error);
        if (roll_pid_i > pid_max) roll_pid_i = pid_max;
        if (roll_pid_i < (pid_max * -1)) roll_pid_i = pid_max * -1;

        pitch_pid_i += pitch_ki * pitch_error;
        if (pitch_pid_i > pid_max) pitch_pid_i = pid_max;
        if (pitch_pid_i < pid_max * -1) pitch_pid_i = pid_max * -1;

        yaw_pid_i += yaw_ki * yaw_error;
        if (yaw_pid_i > yaw_pid_max) yaw_pid_i = yaw_pid_max;
        if (yaw_pid_i < yaw_pid_max * -1) yaw_pid_i = yaw_pid_max * -1;
      }

      // Derivate
      roll_pid_d = roll_kd * (roll_error - roll_previous_error);
      pitch_pid_d = pitch_kd * (pitch_error - pitch_previous_error);
      yaw_pid_d = yaw_kd * (yaw_error - yaw_previous_error);
      
      /* Save Previous Error */
      roll_previous_error = roll_error;
      pitch_previous_error = pitch_error;
      yaw_previous_error = yaw_error;
    }

    // ROLL
    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    if (roll_PID > pid_max) roll_PID = pid_max;
    else if (roll_PID < pid_max * -1)
      roll_PID = pid_max * -1;

    // PITCH
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
    if (pitch_PID > pid_max) pitch_PID = pid_max;
    else if (pitch_PID < pid_max * -1)
      pitch_PID = pid_max * -1;

    // YAW
    yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;
    if (yaw_PID > yaw_pid_max) yaw_PID = yaw_pid_max;
    else if (yaw_PID < yaw_pid_max * -1)
      yaw_PID = yaw_pid_max * -1;

    /* Set the throttle PID for each motor */
    pid_throttle_L_F = throttle + roll_PID + pitch_PID - yaw_PID;
    pid_throttle_R_F = throttle - roll_PID + pitch_PID + yaw_PID;
    pid_throttle_L_B = throttle + roll_PID - pitch_PID + yaw_PID;
    pid_throttle_R_B = throttle - roll_PID - pitch_PID - yaw_PID;

    regulateThrottle();
  } else {
    reset();
  }
}
