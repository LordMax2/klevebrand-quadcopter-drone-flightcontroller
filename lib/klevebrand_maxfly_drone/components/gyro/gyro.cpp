#include "gyro.h"

void Gyro::setReports()
{
  if (!bno08x.enableReport(report_type, report_interval_us))
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

  delay(2000);
}

void Gyro::printYawPitchRoll() {
  Serial.print(yaw_pitch_roll.yaw);
  Serial.print("\t");
  Serial.print(yaw_pitch_roll.pitch);
  Serial.print("\t");
  Serial.println(yaw_pitch_roll.roll);
}

void Gyro::reload()
{
  if (bno08x.getSensorEvent(&sensor_value))
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensor_value.sensorId)
    {
    case SH2_ARVR_STABILIZED_RV:
      quaternionToEulerRv(&sensor_value.un.arvrStabilizedRV, &yaw_pitch_roll, true);
      break;
    case SH2_GYRO_INTEGRATED_RV:
      quaternionToEulerGi(&sensor_value.un.gyroIntegratedRV, &yaw_pitch_roll, true);
      break;
    }
  }
}

void Gyro::quaternionToEulerRv(sh2_RotationVectorWAcc_t *rotational_vector, YawPitchRoll_t *yaw_pitch_roll, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, yaw_pitch_roll, degrees);
}

void Gyro::quaternionToEulerGi(sh2_GyroIntegratedRV_t *rotational_vector, YawPitchRoll_t *yaw_pitch_roll, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, yaw_pitch_roll, degrees);
}

void Gyro::quaternionToEuler(float qr, float qi, float qj, float qk, YawPitchRoll_t *yaw_pitch_roll, bool degrees = false)
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  yaw_pitch_roll->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  yaw_pitch_roll->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yaw_pitch_roll->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    yaw_pitch_roll->yaw *= RAD_TO_DEG;
    yaw_pitch_roll->pitch *= RAD_TO_DEG;
    yaw_pitch_roll->roll *= RAD_TO_DEG;
  }
}