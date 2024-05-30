
#include <Wire.h>
#include <Kalman.h>
#include <PID_v1.h>

#define RESTRICT_PITCH

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

double Setpoint, Output;
double Kp = 2 , Ki = .01, Kd = .09;
float P_val, I_val, D_val;
PID myPID(&kalAngleX, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int Write = 0;


//Motor Driver pins
int RF = 5;
int RB = 6;
int LF = 7;
int LB = 8;
int R_pwm = 9;
int L_pwm = 10;

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);




#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  gyroXangle = roll;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);


  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);

  Setpoint = 1.25;
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  P_val = analogRead(A0);
  I_val = analogRead(A1);
  D_val = analogRead(A2);

  P_val = map(P_val, 0, 1023 , 0 , 20);
  I_val = map(I_val, 0, 1023 , 0 , 1.5);
  D_val = map(D_val, 0, 1023 , 0 , 20);

  //  Kp = P_val;
  //  Ki = I_val;
  //  Kd = D_val;

  myPID.Compute();
  if (kalAngleX < 40 || kalAngleX > -40) {
    MotorControle();
  }
  if (kalAngleX > 40 || kalAngleX < -40) {
    analogWrite(R_pwm, 0);
    analogWrite(L_pwm, 0);
  }

  /* Print Data */

  Serial.print(" ");
  Serial.print(kalAngleX);
  Serial.print(" ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.print(" ");
  Serial.println(Kd);


  delay(5);
}

void MotorControle() {

  if (kalAngleX > Setpoint) { //Move Forward
    float Write = map(Output , -300, 300, 1, 255);
    Write = map(Output, -255 , 255, 1, 255);
    Write = abs(255 - Write);
    digitalWrite(R_pwm, LOW);
    digitalWrite(L_pwm, LOW);
    analogWrite(R_pwm, Write);
    analogWrite(L_pwm, Write);
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
  }

  if (kalAngleX <= Setpoint) { //Move Backward
    float Write = map(Output , -300, 300, 1, 255);
    digitalWrite(R_pwm, LOW);
    digitalWrite(L_pwm, LOW);
    analogWrite(R_pwm, Write);
    analogWrite(L_pwm, Write);
    digitalWrite(RB, HIGH);
    digitalWrite(RF, LOW);
    digitalWrite(LB, HIGH);
    digitalWrite(LF, LOW);
  }

  Serial.print(Write);
  Serial.print(" ");
  Serial.print(Output);
}
