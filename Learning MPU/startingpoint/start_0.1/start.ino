// the idea of low pass filter from https://www.youtube.com/watch?v=IKgqDzfKNW0&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&index=8
// very simple idea put more effective ---> note p1 + p2 = 1 in p1(thetaOld) + p2(thetaNew) = thetaNew



// Basic demo for accelerometer readings from Adafruit MPU6050
#include<I2Cdev.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//#define CHANGE_OFFSET_VALUES //UNcomment this line to change the offset values

int LEDpin = A0;

int Servopin=5;
bool blinkState = true;
int dt = 100;
float theta = 0;
float phi= 0;

float thetaAn =0;
float phiAn =0;
// after the filter 
float thetaOld= 0;
float thetaNew;
float phiOld = 0;
float phiNew; 
float thetaG = 0;
float phiG = 0;
float thetafinal = 0;
float phifinal = 0;
unsigned long millisOld;
float TimeDiff; 

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // puse untill serial come up
  }

  // Try to initialize! --> check if the sensor is conected probably 
  Serial.print("Initializing I2C devices...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  #ifdef CHANGE_OFFSET_VALUES
  #endif
  millisOld = millis();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  pinMode(LEDpin, OUTPUT);

  delay(dt);
}
//////////////////////////////////////  loop /////////////////////////
void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  thetaAn = a.acceleration.x/9.81;
  phiAn = a.acceleration.y/9.81;
  if(thetaAn > 1){
     thetaAn = 1;
    }
  else if(thetaAn < -1){
    thetaAn =-1;
   }

   if(phiAn > 1){
     phiAn = 1;
    }
  else if(phiAn < -1){
    phiAn =-1;
   }
  theta = asin(thetaAn)*(180./3.14159265359); //pitch angle
  phi = asin(phiAn)*(180./3.14159265359); // yaw angle
  
//add filter 
  thetaNew = .92*thetaOld + .08* theta;
  phiNew = .92*phiOld + .08* phi;
  /* Print out the values */
///gyro angles
  TimeDiff = (millis() - millisOld )/1000.; //millis comes with milli second delay
  millisOld = millis();
  thetaG -= (g.gyro.y)*(180./PI)*TimeDiff;
  phiG += (g.gyro.x)*(180./PI)*TimeDiff;

  thetafinal = (thetafinal-(g.gyro.y)*(180./PI)*TimeDiff)*.95 +theta*.05;
  phifinal = (phifinal + (g.gyro.x)*(180./PI)*TimeDiff)*.95 +phi*.05;
   
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.println(a.acceleration.z);
  
//  Serial.print(" ");
//  Serial.print(theta);
//  Serial.print(" ");
//  Serial.print(phi); 
//  Serial.print(" ");
//  Serial.print(thetaG);
//  Serial.print(" ");
//  Serial.print(phiG);
//  Serial.print(" ");
//  
//
  Serial.print(thetafinal);
//  Serial.print(" ");
  Serial.println(phifinal);
  
//  Serial.print(",");
//  Serial.print(" ");
//  Serial.print(g.gyro.z);
//  Serial.println(" ");

  thetaOld = thetaNew;
  phiOld = phiNew;

  blinkState = !blinkState;
  digitalWrite(LEDpin, blinkState);
  delay(dt);
  
}
