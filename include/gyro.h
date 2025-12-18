#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#define BNO_RESET_PIN 10
#define BNO_REPORT_INTERVAL 2500

struct YawPitchRoll_t
{
    float yaw;
    float pitch;
    float roll;
    long timestamp_milliseconds;
};

class Gyro
{
public:
    void setup();
    bool reload();
    void printYawPitchRoll();
    void reset();

    float yaw() const
    {
        return yaw_pitch_roll.yaw;
    }
    float pitch() const
    {
        return yaw_pitch_roll.pitch * -1;
    }
    float roll() const
    {
        return yaw_pitch_roll.roll * -1;
    }
    long timestamp_milliseconds() const
    {
        return yaw_pitch_roll.timestamp_milliseconds;
    }
    bool setReportModeEuler();
    bool setReportModeAcro();

private:
    Adafruit_BNO08x bno08x = Adafruit_BNO08x(BNO_RESET_PIN);
    sh2_SensorValue_t sensor_value;
    YawPitchRoll_t yaw_pitch_roll;
    YawPitchRoll_t quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false);
    YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);
};

#endif