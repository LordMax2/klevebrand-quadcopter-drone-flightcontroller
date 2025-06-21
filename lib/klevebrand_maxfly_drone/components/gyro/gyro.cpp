#include "gyro.h"

void Gyro::setReports()
{
  if (!bno08x.enableReport(report_type, report_interval_us))
  {
    Serial.println("Could not enable stabilized remote vector...");
  }
}

bool Gyro::setReportModeEuler()
{
  return bno08x.enableReport(SH2_ARVR_STABILIZED_RV, report_interval_us);
}

bool Gyro::setReportModeAcro()
{
  return bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval_us);
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

  setReportModeEuler();

  delay(2000);
}

void Gyro::printYawPitchRoll()
{
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
    switch (sensor_value.sensorId)
    {
    case SH2_ARVR_STABILIZED_RV: // Autolevel mode
      YawPitchRoll_t yaw_pitch_roll = quaternionsToYawPitchRoll(&sensor_value.un.arvrStabilizedRV, true);

      Gyro::yaw_pitch_roll.yaw = yaw_pitch_roll.yaw;
      Gyro::yaw_pitch_roll.pitch = yaw_pitch_roll.pitch;
      Gyro::yaw_pitch_roll.roll = yaw_pitch_roll.roll;
      break;
    case SH2_GYROSCOPE_CALIBRATED: // Acro mode
      float gyro_roll = sensor_value.un.gyroscope.x;
      float gyro_pitch = sensor_value.un.gyroscope.y;
      float gyro_yaw = sensor_value.un.gyroscope.z;

      Gyro::yaw_pitch_roll.yaw = gyro_yaw * 57.2958;
      Gyro::yaw_pitch_roll.pitch = gyro_pitch * 57.2958;
      Gyro::yaw_pitch_roll.roll = gyro_roll * 57.2958;
      break;
    }
  }
}

YawPitchRoll_t Gyro::quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool degrees = false)
{
  return quaternionsToYawPitchRoll(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, degrees);
}

YawPitchRoll_t Gyro::quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool degrees = false)
{
  YawPitchRoll_t yaw_pitch_roll;

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  yaw_pitch_roll.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  yaw_pitch_roll.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yaw_pitch_roll.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    yaw_pitch_roll.yaw *= RAD_TO_DEG;
    yaw_pitch_roll.pitch *= RAD_TO_DEG;
    yaw_pitch_roll.roll *= RAD_TO_DEG;
  }

  return yaw_pitch_roll;
}
