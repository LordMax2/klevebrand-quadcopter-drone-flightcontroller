#ifndef TREST_DRONE_H
#define TREST_DRONE_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_BNO08x.h>

#define BNO_RESET_PIN -1
#define BNO_REPORT_INTERVAL 5000

#define RADIO_CE_PIN   9
#define RADIO_CSN_PIN 10

#define PID_THROTTLE_THRESHOLD 1050
#define PID_UPDATE_INTERVAL 1050

enum FlightMode {
  acro = 1,
  autoLevel = 2,
  holdPosition = 3
};

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

class PID {
  public:
    void calculate(float throttle, bool launchMode, float x, float y, float z);
    void regulateThrottle();
    void reset();
    long previousTimer;
    /* Roll PID */
    float roll_PID, pid_throttle_L_F, pid_throttle_L_B, pid_throttle_R_F, pid_throttle_R_B, roll_error, roll_previous_error;
    float roll_pid_p = 0;
    float roll_pid_i = 0;
    float roll_pid_d = 0;
    /* Roll PID Constants */
    double roll_kp = 1;
    double roll_ki = 0.01;
    double roll_kd = 40;
    float roll_desired_angle = 0;

    /* Pitch PID */
    float pitch_PID, pitch_error, pitch_previous_error;
    float pitch_pid_p = 0;
    float pitch_pid_i = 0;
    float pitch_pid_d = 0;
    /* Pitch PID Constants */
    double pitch_kp = roll_kp;
    double pitch_ki = roll_ki;
    double pitch_kd = roll_kd;
    float pitch_desired_angle = 0;

    /* Yaw PID */
    float yaw_PID, yaw_error, yaw_previous_error;
    float yaw_pid_p = 0;
    float yaw_pid_i = 0;
    float yaw_pid_d = 0;
    /* Pitch PID Constants */
    double yaw_kp = 0; //0.5
    double yaw_ki = 0;
    double yaw_kd = 0; //5
    float yaw_desired_angle = 0;

    int yaw_pid_max = 200;
    int pid_max = 400;

    float pitchOffset = 0, rollOffset = 0;
  private:
    void updateIntegral();
    void resetIntegral();
    void constrainPID(float &pid_value);
    void constrainYawPID(float &yaw_PID);
};

class Gyro {
  public:
    void setup();
    void setReports();
    void update();
    euler_t ypr;
    Adafruit_BNO08x bno08x = Adafruit_BNO08x(BNO_RESET_PIN);
    sh2_SensorValue_t sensorValue;
    sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
    long reportIntervalUs = BNO_REPORT_INTERVAL;
  private:
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);
};

class ReciverData {
  public:
    int inputThrottle;
    int yawDesiredAngle;
    int rollDesiredAngle;
    int pitchDesiredAngle;
    FlightMode flightMode;
    float lastRecivedMessageMillis = millis();
};

class FeedbackData {
  public:
    float yaw;
    float pitch;
    float roll;
    FlightMode flightMode;
};

class Reciver {
  public:
    void setup();
    void recive();
    void transmit(float roll, float pitch, float yaw, FlightMode flightMode);
    RF24 radio = RF24(RADIO_CE_PIN, RADIO_CSN_PIN);
    const uint64_t pAddress = 0xB00B1E5000LL;
    const uint64_t fAddress = 0xC3C3C3C3C3LL;
    ReciverData reciverData;
    FeedbackData feedbackData;
};

class Drone {
  public:
    void setup();
    void setupMotors();
    void regulateInputsPID();
    void setZero();
    void runMotors();
    void stopMotors();
    void calculatePID();
    bool lostConnection();
    void updateGyro();
    euler_t getGyroYpr();
    void reciverRecive();
    void resetPID();
    void regulateThrottlePID();
    bool launchMode = true;
    bool setZeroBool = true;
    Servo motorLF;
    Servo motorRF;
    Servo motorLB;
    Servo motorRB;
  private:
    Reciver reciver;
    Gyro gyro;
    PID pid;
};

#endif