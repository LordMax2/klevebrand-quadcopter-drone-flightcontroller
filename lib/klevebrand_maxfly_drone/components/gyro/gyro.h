#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#define BNO_RESET_PIN -1
#define BNO_REPORT_INTERVAL 2000

struct YawPitchRoll_t
{
  float yaw;
  float pitch;
  float roll;
};

class Gyro
{
public:
    Adafruit_BNO08x bno08x = Adafruit_BNO08x(BNO_RESET_PIN);
    float yaw() const
    {
        return yawPitchRoll.yaw;
    }
    float pitch() const
    {
        return yawPitchRoll.pitch;
    }
    float roll() const
    {
        return yawPitchRoll.roll;
    }

    void setup();
    void setReports();
    void update();

private:
    sh2_SensorValue_t sensorValue;
    sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
    YawPitchRoll_t yawPitchRoll;
    long reportIntervalUs = BNO_REPORT_INTERVAL;

    void quaternionToEuler(float qr, float qi, float qj, float qk, YawPitchRoll_t *ypr, bool degrees = false);
    void quaternionToEulerRv(sh2_RotationVectorWAcc_t *rotational_vector, YawPitchRoll_t *ypr, bool degrees = false);
    void quaternionToEulerGi(sh2_GyroIntegratedRV_t *rotational_vector, YawPitchRoll_t *ypr, bool degrees = false);
};

#endif