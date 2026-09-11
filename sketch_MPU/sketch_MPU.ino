#include <Wire.h>
#include "mpu6500.h"

bfs::Mpu6500 MPU;

float yAccel;
float tilt;


void setup() {
  Serial.begin(9600); //Begin Serial for seeing data

  // Start I2C
  Wire.begin(22, 21);      // SDA = 22, SCL = 21
  Wire.setClock(400000);   // 400 kHz I2C speed

  // Tell the library which I2C bus and address to use
  MPU.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

  // Start the MPU6500
  if (MPU.Begin()) {
    Serial.println("MPU6500 connected");
  } else {
    Serial.println("MPU6500 failed");
  }
}

void loop() {

  if (MPU.Read()) {

    yAccel = MPU.accel_y_mps2();
    tilt = asin(yAccel/9.81)*180/PI;
    Serial.println(tilt);

    
  }

  delay(200);
}