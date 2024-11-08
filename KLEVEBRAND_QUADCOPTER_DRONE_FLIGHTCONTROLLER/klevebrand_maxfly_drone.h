#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_BNO08x.h>

#define BNO_RESET_PIN -1
#define BNO_REPORT_INTERVAL 2000

#define RADIO_CE_PIN   9
#define RADIO_CSN_PIN 10

#define RADIO_TIMEOUT_DEFINITION_MILLISECONDS 400

#define PID_THROTTLE_THRESHOLD 1050
#define PID_UPDATE_INTERVAL 10
#define PID_MAX 400

#define THROTTLE_MINIMUM 1000
#define THROTTLE_MAXIMUM 2000

enum FlightMode_t {
  acro = 1,
  autoLevel = 2,
  holdPosition = 3
};

struct YawPitchRoll_t {
  float yaw;
  float pitch;
  float roll;
};

class PID {
  public:
    void calculate(float throttle, bool launchMode, float gyroRoll, float gyroPitch, float gyroYaw);
    void reset();
    long previousTimer;
    [[nodiscard]] float pid_throttle_L_F(float throttle) const {
      return constrain(throttle + roll_PID() - pitch_PID() - yaw_PID(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    [[nodiscard]] float pid_throttle_L_B(float throttle) const {
      return constrain(throttle + roll_PID() + pitch_PID() + yaw_PID(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    [[nodiscard]] float pid_throttle_R_F(float throttle) const {
      return constrain(throttle - roll_PID() - pitch_PID() + yaw_PID(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    [[nodiscard]] float pid_throttle_R_B(float throttle) const {
      return constrain(throttle - roll_PID() + pitch_PID() - yaw_PID(), THROTTLE_MINIMUM, THROTTLE_MAXIMUM);
    }
    float pitchOffset = 0, rollOffset = 0;
  private:
    void updateIntegral();
    void resetIntegral();
    
    /* Roll PID */
    [[nodiscard]] float roll_PID() const {
      return constrain(roll_pid_p() + roll_pid_i + roll_pid_d(), -PID_MAX, PID_MAX);
    }
    float roll_error, roll_previous_error;
    [[nodiscard]] float roll_pid_p() const {
      return roll_kp * roll_error;
    }
    float roll_pid_i = 0;
    [[nodiscard]] float roll_pid_d() const {
      return roll_kd * (roll_error - roll_previous_error);
    }
    /* Roll PID Constants */
    double roll_kp = 1;
    double roll_ki = 0.01;
    double roll_kd = 10;
    float roll_desired_angle = 0;

    /* Pitch PID */
    [[nodiscard]] float pitch_PID() const {
      return constrain(pitch_pid_p() + pitch_pid_i + pitch_pid_d(), -PID_MAX, PID_MAX);
    }
    float pitch_error, pitch_previous_error;
    [[nodiscard]] float pitch_pid_p() const {
      return pitch_kp * pitch_error;
    }
    float pitch_pid_i = 0;
    [[nodiscard]] float pitch_pid_d() const {
      return pitch_kd * (pitch_error - pitch_previous_error);
    }
    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;
    float pitch_desired_angle = 0;

    /* Yaw PID */
    [[nodiscard]] float yaw_PID() const {
      return constrain(yaw_pid_p() + yaw_pid_i + yaw_pid_d(), -PID_MAX, PID_MAX);
    }
    float yaw_error, yaw_previous_error;
    [[nodiscard]] float yaw_pid_p() const {
      return yaw_kp * yaw_error;
    }
    float yaw_pid_i = 0;
    [[nodiscard]] float yaw_pid_d() const {
      return yaw_kd * (yaw_error - yaw_previous_error);
    }
    /* Pitch PID Constants */
    double yaw_kp = 0; //0.5
    double yaw_ki = 0;
    double yaw_kd = 0; //5
    float yaw_desired_angle = 0;
};

class Gyro {
  public:
    void setup();
    void setReports();
    void update();
    YawPitchRoll_t yawPitchRoll;
    Adafruit_BNO08x bno08x = Adafruit_BNO08x(BNO_RESET_PIN);
  private:
    void quaternionToEuler(float qr, float qi, float qj, float qk, YawPitchRoll_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, YawPitchRoll_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, YawPitchRoll_t* ypr, bool degrees = false);
    sh2_SensorValue_t sensorValue;
    sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
    long reportIntervalUs = BNO_REPORT_INTERVAL;
};

class ReciverData {
  public:
    int inputThrottle;
    int yawDesiredAngle;
    int rollDesiredAngle;
    int pitchDesiredAngle;
    FlightMode_t flightMode;
    float lastRecivedMessageMillis = millis();
};

class FeedbackData {
  public:
    float yaw;
    float pitch;
    float roll;
    FlightMode_t flightMode;
};

class Reciver {
  public:
    void setup();
    void recive();
    void transmit(float roll, float pitch, float yaw, FlightMode_t flightMode);
    RF24 radio = RF24(RADIO_CE_PIN, RADIO_CSN_PIN);
    ReciverData reciverData;
    FeedbackData feedbackData;
  private:
    const uint64_t pAddress = 0xB00B1E5000LL;
    const uint64_t fAddress = 0xC3C3C3C3C3LL;
};

class Drone {
  public:
    void setup();
    void setPitchAndRollGyroOffsetAndDefineCurrentAngleAsZero();
    void runMotors();
    void stopMotors();
    void updateGyro();
    void reciverRecive();
    void printThrottle();
    void resetPID();
    bool lostConnection();
    void calculatePID();
  private:
    Reciver reciver;
    Gyro gyro;
    PID pid;
    Servo motorLeftFront;
    Servo motorRightFront;
    Servo motorLeftBack;
    Servo motorRightBack;
    bool launchMode = true;
    bool setZeroBool = true;
    [[nodiscard]] float getInputThrottle() const {
      return reciver.reciverData.inputThrottle;
    }
    void constrainReciverInputs();
    void setupMotors();
    YawPitchRoll_t getGyroYawPitchRoll();
};

#endif