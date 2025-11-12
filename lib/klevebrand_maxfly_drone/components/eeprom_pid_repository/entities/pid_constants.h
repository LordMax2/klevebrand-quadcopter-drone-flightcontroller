#ifndef PID_CONSTANTS_H
#define PID_CONSTANTS_H

struct PidConstants
{
    PidConstants() :    yaw_kp(0.0f), yaw_ki(0.0f), yaw_kd(0.0f),
                        pitch_kp(0.0f), pitch_ki(0.0f), pitch_kd(0.0f),
                        roll_kp(0.0f), roll_ki(0.0f), roll_kd(0.0f) {}

    PidConstants(float yaw_kp, float yaw_ki, float yaw_kd,
                 float pitch_kp, float pitch_ki, float pitch_kd,
                 float roll_kp, float roll_ki, float roll_kd)
        : yaw_kp(yaw_kp), yaw_ki(yaw_ki), yaw_kd(yaw_kd),
          pitch_kp(pitch_kp), pitch_ki(pitch_ki), pitch_kd(pitch_kd),
          roll_kp(roll_kp), roll_ki(roll_ki), roll_kd(roll_kd) {}
    
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float roll_kp;
    float roll_ki;
    float roll_kd;

    bool isValid() 
    {
      bool everything_is_not_zero = yaw_kp != 0.0f && yaw_ki != 0.0f && yaw_kd != 0.0f &&
             pitch_kp != 0.0f && pitch_ki != 0.0f && pitch_kd != 0.0f &&
             roll_kp != 0.0f && roll_ki != 0.0f && roll_kd != 0.0f;

      bool nothing_is_nan = !isnan(yaw_kp) || !isnan(yaw_ki) || !isnan(yaw_kd) ||
            !isnan(pitch_kp) || !isnan(pitch_ki) || !isnan(pitch_kd) ||
            !isnan(roll_kp) || !isnan(roll_ki) || !isnan(roll_kd);

      return everything_is_not_zero && nothing_is_nan;
    }
};

#endif // PID_CONSTANTS_H