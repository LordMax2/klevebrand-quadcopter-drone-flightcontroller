#include "gyro.h"

void Gyro::setReports()
{
  if (!bno08x.enableReport(reportType, reportIntervalUs))
  {
    Serial.println("Could not enable stabilized remote vector...");
  }
}

void Gyro::setup()
{
  Wire.begin();
  
  Serial.println("Setting up gyroscope");
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to connect to BNO085...");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("BNO085 set up!");

  setReports();

  delay(5000);
}

void Gyro::printYawPitchRoll() {
  Serial.print(yawPitchRoll.yaw);
  Serial.print("\t");
  Serial.print(yawPitchRoll.pitch);
  Serial.print("\t");
  Serial.println(yawPitchRoll.roll);
}

void Gyro::update()
{
  if (bno08x.getSensorEvent(&sensorValue))
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId)
    {
    case SH2_ARVR_STABILIZED_RV:
      quaternionToEulerRv(&sensorValue.un.arvrStabilizedRV, &yawPitchRoll, true);
      break;
    case SH2_GYRO_INTEGRATED_RV:
      quaternionToEulerGi(&sensorValue.un.gyroIntegratedRV, &yawPitchRoll, true);
      break;
    }
  }
}

void Gyro::quaternionToEulerRv(sh2_RotationVectorWAcc_t *rotational_vector, YawPitchRoll_t *yawPitchRoll, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, yawPitchRoll, degrees);
}

void Gyro::quaternionToEulerGi(sh2_GyroIntegratedRV_t *rotational_vector, YawPitchRoll_t *yawPitchRoll, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, yawPitchRoll, degrees);
}

void Gyro::quaternionToEuler(float qr, float qi, float qj, float qk, YawPitchRoll_t *yawPitchRoll, bool degrees = false)
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  yawPitchRoll->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  yawPitchRoll->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yawPitchRoll->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    yawPitchRoll->yaw *= RAD_TO_DEG;
    yawPitchRoll->pitch *= RAD_TO_DEG;
    yawPitchRoll->roll *= RAD_TO_DEG;
  }
}