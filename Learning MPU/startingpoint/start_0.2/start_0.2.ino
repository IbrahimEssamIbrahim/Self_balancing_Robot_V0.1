
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double AccAngle =0,GyraoAngle =0, InputAngle;
float  Lowpass = .05;
float millisOld; 
int Rled = A0, Gled = A1;
void setup() {

    Serial.begin(9600);
    accelgyro.initialize();
    millisOld = 0;
    // verify connection
    Serial.println("Testing device connections...");
    if(!accelgyro.testConnection()){
      Serial.print("Error finding IMU sensor.");
      while(true);
    }
    

    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    accelgyro.setXGyroOffset(accelgyro.getXGyroOffset());
    accelgyro.setYGyroOffset(accelgyro.getYGyroOffset());
    accelgyro.setZGyroOffset(accelgyro.getZGyroOffset());
    Serial.print(accelgyro.getXAccelOffset()); Serial.print(" "); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print(" "); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print(" "); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print(" "); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print(" "); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.println(); // 0


    // configure Arduino LED pin for output
    pinMode(Rled, OUTPUT);
    pinMode(Gled, OUTPUT);
    

}

void loop() {
    // read raw accel/gyro measurements from IMU sensor.
    accelgyro.getMotion6(&aX, &aY, &aZ, &gx, &gy, &gz);

        // display tab-separated accel/gyro x/y/z values
       Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
     AccAngle  = atan2(aY, aZ) * RAD_TO_DEG;
     GyraoAngle +=  gx*(millis() - millisOld)* RAD_TO_DEG;
     millisOld = millis();
     InputAngle = (1-Lowpass)*GyraoAngle + Lowpass*AccAngle
     
}
