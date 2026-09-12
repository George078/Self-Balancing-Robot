#include <Wire.h>  // Call libraries for I2C and MPU
#include "mpu6500.h"

bfs::Mpu6500 MPU; // Create MPU object

double yAccel; // Declaring variables which will later be used to store MPU data
double tilt;

int PWMA = 23; // Initialising MOTOR A driver pins 
int AIN1=27;
int AIN2=26;

int PWMB = 19; // Initialising MOTOR B driver pins 
int BIN1=13;
int BIN2=14;

double dt, lastTime, filteredTilt, aim = 0; // aim = desired angle of lean for the robot (0 being stood vertical)
double integral, previous, output = 0;
double kp, ki, kd;

void setup()
{
  //PID Control Gains

  kp = 3;
  ki = 0;  
  kd = 0;

  Serial.begin(9600); // Initialise communication via serial at 9600 baud

  pinMode(PWMA, OUTPUT); // Setting MOTOR pinmodes to output 
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  analogWrite(PWMA, 0); // Ensuring no motor spin on start-up
  analogWrite(PWMB, 0);

  // Start I2C
  Wire.begin(22, 21);      // SDA = 22, SCL = 21
  Wire.setClock(400000);   // 400 kHz I2C speed

  // Tell the library which I2C bus and address to use
  MPU.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

  // Start the MPU6500
  MPU.Begin();

  delay(5000); // Pausing for 5secs to allow for robot to be placed on surface

  lastTime = millis(); // setting a waypoint for MILLIS 
}

void loop() {

  //Setting up millis and calculating dt (in seconds)
  double now = millis();
  dt = (now - lastTime)/1000.00;
  lastTime = now;

  //Take readings from MPU
  if (MPU.Read()) {

    yAccel = MPU.accel_y_mps2(); // storing accelerometer Y value

    // Prevent asin() receiving values outside -1 to +1
    yAccel = constrain(yAccel, -9.81, 9.81);

    tilt = asin(yAccel/9.81)*180/PI; // converting to degrees

    double gyroY = MPU.gyro_y_radps() * 180.0 / PI; // converting Gyro Y to degrees


    filteredTilt = 0.98 * (filteredTilt + gyroY * dt) + 0.02 * tilt; // calculate tilt with complementary filter


    if (abs(filteredTilt) > 35) {

     analogWrite(PWMA, 0);
     analogWrite(PWMB, 0);
     integral = 0;
     return;

    }


    double error = aim - filteredTilt;  // error calc

    // PID calc
    output = pid(error);

    // Motor power as magnitude of PID output
    int motorPowerA = 0;
    int motorPowerB = 0;

    // Small deadzone around vertical
    if (abs(output) > 2)
    {
      motorPowerA = constrain(abs(output) + 30, 0, 250);
      motorPowerB = constrain(abs(output) + 25, 0, 250);
    }

    // One direction case
    if (output > 2)
    {
      analogWrite(PWMA, motorPowerA);
      analogWrite(PWMB, motorPowerB);

      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);

      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);

    }

    // Other direction case
    else if (output < -2)
    {
      analogWrite(PWMA, motorPowerA);
      analogWrite(PWMB, motorPowerB);

      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);

      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }

    // PID output is zero (not going to happen)
    else
    {
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
    }

    Serial.print("Accel Tilt: "); //output values for debugging and tuning
    Serial.print(tilt);

    Serial.print(" Gyro Y: ");
    Serial.print(gyroY);

    Serial.print(" Filtered: ");
    Serial.print(filteredTilt);

    Serial.print("  Motor A: ");
    Serial.print(motorPowerA);

    Serial.print("  Motor B: ");
    Serial.println(motorPowerB);

    

  }

  else {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }

}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}