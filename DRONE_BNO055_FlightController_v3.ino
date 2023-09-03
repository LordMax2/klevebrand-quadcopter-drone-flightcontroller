#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>

#define CE_PIN   9
#define CSN_PIN 10

// Create Radio and define pins
RF24 radio(CE_PIN, CSN_PIN);

// Create Radio channels
const uint64_t pAddress = 0xB00B1E5000LL;
const uint64_t fAddress = 0xC3C3C3C3C3LL;

// Create a boolean for connection lost
bool lostConnection = false;

// Flight modes
bool acro, autoLevel = true, holdPosition, launchMode = true;

//Radio recived array
float recivedDataArray[8];
float feedbackArray[5];

Servo motor_LF;
Servo motor_RF;
Servo motor_LB;
Servo motor_RB;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

imu::Vector<3> gyro;

bool setZeroBool = true;

float previousMessageMillis;

float input_throttle = 1000;
float input_yaw = 0;
float input_pitch = 0;
float input_roll = 0;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!Serial);
  printf_begin();

  if (!bno.begin())
  {
    Serial.print("Error, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  int8_t temp = bno.getTemp();
  bno.setExtCrystalUse(true);

  delay(1000);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pAddress);
  radio.openWritingPipe(fAddress);

  radio.printDetails();

  //radio.printDetails();
  radio.startListening();

  delay(5000);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  motor_LF.attach(5);
  motor_RF.attach(6);
  motor_LB.attach(7);
  motor_RB.attach(8);

  motor_LF.writeMicroseconds(1000);
  motor_RF.writeMicroseconds(1000);
  motor_LB.writeMicroseconds(1000);
  motor_RB.writeMicroseconds(1000);
}

void loop() {
  if (autoLevel == true || holdPosition == true) {
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Serial.println("VECTOR_EULER");
  } else if (acro == true) {
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    //Serial.println("VECTOR_GYROSCOPE");
  } else {
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Serial.println("Else");
  }
  GetRadio();
  if (lostConnection == true) {
    PID_reset();
    motor_LF.writeMicroseconds(1000);
    motor_RF.writeMicroseconds(1000);
    motor_LB.writeMicroseconds(1000);
    motor_RB.writeMicroseconds(1000);
    Serial.println("LOST CONNECTION!");
  } else {
    GoodConnection();
    motor_LF.writeMicroseconds(pid_throttle_L_F);
    motor_RF.writeMicroseconds(pid_throttle_R_F);
    motor_LB.writeMicroseconds(pid_throttle_L_B);
    motor_RB.writeMicroseconds(pid_throttle_R_B);
  }
  //sendFeedback(gyro.x(), gyro.y(), gyro.z());
}

void GetRadio() {
  if (radio.available()) {
    radio.read(&recivedDataArray, sizeof(recivedDataArray));
    //Serial.println("Recieved array:");
    previousMessageMillis = millis();
    for (byte i = 0; i < 4; i++) {
      //Serial.println(recivedDataArray[i]);
    }
    input_throttle = recivedDataArray[0];
    yaw_desired_angle += recivedDataArray[1];
    roll_desired_angle = recivedDataArray[2] + rollOffset;
    pitch_desired_angle = -recivedDataArray[3] + pitchOffset;

    if (recivedDataArray[4] == 1) {
      acro = true;
      autoLevel = false;
      holdPosition = false;
    } else if (recivedDataArray[4] == 2) {
      acro = false;
      autoLevel = true;
      holdPosition = false;
    } else if (recivedDataArray[4] == 3) { // ICKE AKTIVERAD ÄNNU, KOM IHÅG
      acro = false;
      autoLevel = true;
      holdPosition = false;
    }
    //Serial.println();
  }
  if (millis() - previousMessageMillis >= 400) {
    lostConnection = true;
  } else {

    lostConnection = false;
  }
}

void GoodConnection() {


  if (yaw_desired_angle > 360) {
    yaw_desired_angle = 0;
  } else if (yaw_desired_angle < 0) {
    yaw_desired_angle = 360;
  }

  if (input_throttle > 1600) {
    input_throttle = 1600;
  } else if (input_throttle < 1000) {
    input_throttle = 1000;
  }

  if (millis() >= 15000) {
    launchMode = false;
  }

  SetZero(gyro.x(), gyro.y(), gyro.z());
  PIDControl(gyro.x(), gyro.y(), gyro.z());



  //print_roll_pitch_yaw(gyro.x(), gyro.y(), gyro.z());
  //printDesiredAngles();
  //print_PID();
  print_throttle();
  //print_detailed_PID();
  //printDesiredYaw(gyro.z());

}

void sendFeedback(float x, float y, float z) {
  feedbackArray[0] = x;
  feedbackArray[1] = z;
  feedbackArray[2] = y;
  if (autoLevel == true) {
    feedbackArray[3] = 1;
  } else if (acro == true) {
    feedbackArray[3] = 2;
  } else if (holdPosition == true) {
    feedbackArray[3] = 3;
  }

  radio.stopListening();
  radio.write( &feedbackArray, sizeof(feedbackArray));
  radio.startListening();
}

void PIDControl(float x, float y, float z) {
  if (input_throttle > 1050) {
    /* PID */
    roll_error = roll_desired_angle - y;
    pitch_error = pitch_desired_angle - z;
    yaw_error = yaw_desired_angle - x;

    //Proportional dowsn't need any delay any change can be used instantly
    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;
    yaw_pid_p = yaw_kp * yaw_error;

    //Integral and Derivative need a delay 10 ms should be enough. We will use an exact delay to speed up the process
    // if you dont use a delay integral will wind up fast
    // Derivitive will not see any substantial change to be usefull

    static unsigned long _ExactTimer;
    if (( millis() - _ExactTimer) >= (10)) {
      _ExactTimer += (10);
      if (( millis() - _ExactTimer) >= (1))_ExactTimer = millis(); // prevents timer windup
      // Integral
      if (launchMode) {
        roll_pid_i = 0;
        pitch_pid_i = 0;
        yaw_pid_i = 0;
      } else {
        roll_pid_i = roll_pid_i + (roll_ki * roll_error);
        if (roll_pid_i > pid_max) roll_pid_i = pid_max;
        if (roll_pid_i < (pid_max * -1)) roll_pid_i = pid_max * -1;

        pitch_pid_i += pitch_ki * pitch_error;
        if (pitch_pid_i > pid_max) pitch_pid_i = pid_max;
        if (pitch_pid_i < pid_max * -1) pitch_pid_i = pid_max * -1;

        yaw_pid_i += yaw_ki * yaw_error;
        if (yaw_pid_i > yaw_pid_max) yaw_pid_i = yaw_pid_max;
        if (yaw_pid_i < yaw_pid_max * -1) yaw_pid_i = yaw_pid_max * -1;

      }



      // Derivate
      roll_pid_d = roll_kd * (roll_error - roll_previous_error);
      pitch_pid_d = pitch_kd * (pitch_error - pitch_previous_error);
      yaw_pid_d = yaw_kd * (yaw_error - yaw_previous_error);
      
      /* Save Previous Error */
      roll_previous_error = roll_error;
      pitch_previous_error = pitch_error;
      yaw_previous_error = yaw_error;
    }
    // ROLL


    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    if (roll_PID > pid_max) roll_PID = pid_max;
    else if (roll_PID < pid_max * -1)
      roll_PID = pid_max * -1;

    // PITCH


    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
    if (pitch_PID > pid_max) pitch_PID = pid_max;
    else if (pitch_PID < pid_max * -1)
      pitch_PID = pid_max * -1;

    // YAW


    yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;
    if (yaw_PID > yaw_pid_max) yaw_PID = yaw_pid_max;
    else if (yaw_PID < yaw_pid_max * -1)
      yaw_PID = yaw_pid_max * -1;

    /* Set the throttle PID for each motor */
    pid_throttle_L_F = input_throttle + roll_PID + pitch_PID - yaw_PID;
    pid_throttle_R_F = input_throttle - roll_PID + pitch_PID + yaw_PID;
    pid_throttle_L_B = input_throttle + roll_PID - pitch_PID + yaw_PID;
    pid_throttle_R_B = input_throttle - roll_PID - pitch_PID - yaw_PID;



    /* Regulate throttle for ESCs */
    //Right front
    if (pid_throttle_R_F < 1100) {
      pid_throttle_R_F = 1100;
    }
    if (pid_throttle_R_F > 2000) {
      pid_throttle_R_F = 2000;
    }

    //Left front
    if (pid_throttle_L_F < 1100) {
      pid_throttle_L_F = 1100;
    }
    if (pid_throttle_L_F > 2000) {
      pid_throttle_L_F = 2000;
    }

    //Right back
    if (pid_throttle_R_B < 1100) {
      pid_throttle_R_B = 1100;
    }
    if (pid_throttle_R_B > 2000) {
      pid_throttle_R_B = 2000;
    }

    //Left back
    if (pid_throttle_L_B < 1100) {
      pid_throttle_L_B = 1100;
    }
    if (pid_throttle_L_B > 2000) {
      pid_throttle_L_B = 2000;
    }
  } else {
    PID_reset();
  }
}

void print_roll_pitch_yaw(float x, float y, float z) {
  Serial.print(z);
  Serial.print(", ");
  Serial.println(y);
}

void printDesiredAngles() {
  Serial.print(roll_desired_angle);
  Serial.print(",");
  Serial.println(pitch_desired_angle);
}


void printDesiredYaw(float yaw) {
  Serial.print(yaw_desired_angle);
  Serial.print(",");
  Serial.println(yaw);
}

void print_throttle() {
  Serial.print(pid_throttle_L_F);
  Serial.print("   ");
  Serial.println(pid_throttle_R_F);
  Serial.print(pid_throttle_L_B);
  Serial.print("   ");
  Serial.println(pid_throttle_R_B);
  Serial.println();
}

void print_PID() {
  Serial.print(roll_PID);
  Serial.print(", ");
  Serial.print(pitch_PID);
  Serial.print(", ");
  Serial.println(yaw_PID);
}

void print_detailed_PID() {
  Serial.print(pitch_pid_p);
  Serial.print(", ");
  Serial.print(pitch_pid_i);
  Serial.print(", ");
  Serial.print(pitch_pid_d);
  Serial.print(", ");
  Serial.println(pitch_PID);
}

void SetZero(float x, float y, float z) {
  if (setZeroBool) {
    pitchOffset = y;
    rollOffset = z;
    yaw_desired_angle = x;
    setZeroBool = false;
  }
}

void PID_reset() {
  pid_throttle_L_F = 1000;
  pid_throttle_L_B = 1000;
  pid_throttle_R_F = 1000;
  pid_throttle_R_B = 1000;

  pitch_PID = 0, roll_PID = 0, yaw_PID = 0;
  roll_error = 0, pitch_error = 0, yaw_error = 0;
  roll_previous_error = 0, pitch_previous_error = 0, yaw_previous_error = 0;

  yaw_pid_p = 0, roll_pid_p = 0, pitch_pid_p = 0;
  yaw_pid_i = 0, roll_pid_i = 0, pitch_pid_i = 0;
  yaw_pid_d = 0, roll_pid_d = 0, pitch_pid_d = 0;
}
