// the idea of low pass filter from https://www.youtube.com/watch?v=IKgqDzfKNW0&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&index=8
// very simple idea put more effective ---> note p1 + p2 = 1 in p1(thetaOld) + p2(thetaNew) = thetaNew




#include<I2Cdev.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>



int LEDpin = A0;

bool blinkState = true;
int dt = 100;

float theta = 0;  //pitch --->raw data
float phi= 0;     //roll  --->raw data

float thetaAn =0; //for if staments
float phiAn =0;

// after the filter 

float thetaG = 0;
float phiG = 0;
float thetafinal = 0;
float phifinal = 0;
float yaw = 0;

unsigned long millisOld; 
float TimeDiff; 

Adafruit_MPU6050 mpu;

void setup(void) {
  
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // puse untill serial come up
  }

  // Try to initialize! --> check if the sensor is conected probably 
//  Serial.print("Initializing I2C devices...");
  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  millisOld = millis();
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  pinMode(LEDpin, OUTPUT);


  delay(dt);
}
//////////////////////////////////////  loop /////////////////////////
void loop() {

  /* Get  sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  thetaAn = a.acceleration.x/9.81;
  phiAn = a.acceleration.y/9.81;
  // the if statment to make sure that arcCos doesn't take a value bigger than 1 or less than -1
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
  

///gyro angles from the equation --> theta = theta + ω*dt   ω angler velocity 
  
  TimeDiff = (millis() - millisOld )/1000.; //millis comes with milli second delay
  millisOld = millis();
  thetaG -= (g.gyro.y)*(180./PI)*TimeDiff;
  phiG += (g.gyro.x)*(180./PI)*TimeDiff;

  thetafinal = (thetafinal-(g.gyro.y)*(180./PI)*TimeDiff)*.9 +theta*.1;
  phifinal = (phifinal + (g.gyro.x)*(180./PI)*TimeDiff)*.9 +phi*.1;

  //controle Servos



  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(thetafinal);
  Serial.print(" ");
  Serial.println(phifinal);
  



  blinkState = !blinkState;
  digitalWrite(LEDpin, blinkState);
  delay(dt);
  
}
