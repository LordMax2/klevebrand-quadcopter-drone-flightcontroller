#include "gyro.h"

bool Gyro::setReportModeEuler()
{
  bool result = bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 4000);

  if (!result)
  {
    Serial.println("FAILED TO SETUP EULER MODE.");
  }

  return result;
}

bool Gyro::setReportModeAcro()
{
  bool result = bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2000);

  if (!result)
  {
    Serial.println("FAILED TO SETUP ACRO MODE.");
  }

  return result;
}

void Gyro::setup()
{
  Serial.println("SETTING UP GYROSCOPE.");
  if (!bno08x.begin_I2C())
  {
    Serial.println("FAILED TO CONNECT TO BNO085...");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("BNO085 SET UP!");

  delay(300);
}

void Gyro::printYawPitchRoll()
{
  Serial.print(yaw_pitch_roll.yaw);
  Serial.print("\t");
  Serial.print(yaw_pitch_roll.pitch);
  Serial.print("\t");
  Serial.println(yaw_pitch_roll.roll);
}

bool Gyro::reload()
{
  if (bno08x.getSensorEvent(&sensor_value))
  {
    if (sensor_value.sensorId == 40)
    {
      YawPitchRoll_t yaw_pitch_roll = quaternionsToYawPitchRoll(&sensor_value.un.arvrStabilizedRV, true);

      Gyro::yaw_pitch_roll.yaw = yaw_pitch_roll.yaw;
      Gyro::yaw_pitch_roll.pitch = yaw_pitch_roll.pitch;
      Gyro::yaw_pitch_roll.roll = yaw_pitch_roll.roll;
    }

    if (sensor_value.sensorId == 2)
    {
      float gyro_roll = sensor_value.un.gyroscope.x;
      float gyro_pitch = sensor_value.un.gyroscope.y;
      float gyro_yaw = sensor_value.un.gyroscope.z;

      Gyro::yaw_pitch_roll.yaw = gyro_yaw * RAD_TO_DEG;
      Gyro::yaw_pitch_roll.pitch = gyro_pitch * RAD_TO_DEG;
      Gyro::yaw_pitch_roll.roll = gyro_roll * RAD_TO_DEG;
    }

    Gyro::yaw_pitch_roll.timestamp_milliseconds = millis();

    return true;
  }

  return false;
}

YawPitchRoll_t Gyro::quaternionsToYawPitchRoll(sh2_RotationVectorWAcc_t *rotational_vector, bool to_degrees)
{
  return quaternionsToYawPitchRoll(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, to_degrees);
}

YawPitchRoll_t Gyro::quaternionsToYawPitchRoll(float qr, float qi, float qj, float qk, bool to_degrees)
{
  YawPitchRoll_t yaw_pitch_roll;

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  yaw_pitch_roll.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  yaw_pitch_roll.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yaw_pitch_roll.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (to_degrees)
  {
    yaw_pitch_roll.yaw *= RAD_TO_DEG;
    yaw_pitch_roll.pitch *= RAD_TO_DEG;
    yaw_pitch_roll.roll *= RAD_TO_DEG;
  }

  return yaw_pitch_roll;
}
