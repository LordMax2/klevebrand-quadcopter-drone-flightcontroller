#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#define BNO_RESET_PIN -1
#define BNO_REPORT_INTERVAL 2500

struct YawPitchRoll_t
{
    float yaw;
    float pitch;
    float roll;
};

class Gyro
{
public:
    void setup();
    void reload();
    void printYawPitchRoll();

    float yaw() const
    {
        return yaw_pitch_roll.yaw;
    }
    float pitch() const
    {
        return yaw_pitch_roll.pitch;
    }
    float roll() const
    {
        return yaw_pitch_roll.roll;
    }

private:
    Adafruit_BNO08x bno08x = Adafruit_BNO08x(BNO_RESET_PIN);
    sh2_SensorValue_t sensor_value;
    sh2_SensorId_t report_type = SH2_ARVR_STABILIZED_RV;
    YawPitchRoll_t yaw_pitch_roll;
    long report_interval_us = BNO_REPORT_INTERVAL;
    YawPitchRoll_t quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false);
    YawPitchRoll_t quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false);

    void setReports();
    bool setReportModeEuler();
    bool setReportModeAcro();
};

#endif