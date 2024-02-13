

#include <Wire.h>
// #include <Adafruit_MPU6050.h>
#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include "encoder.h"

MPU6050 mpu(Wire);

int motor1Pin1 = 6;
int motor1Pin2 = 7;
int motor2Pin1 = 4;
int motor2Pin2 = 5;
int motorDirection = 1;
double setpoint = 0;
double Kp = 20;//10.532;
double Ki = 0;//18.338;
double Kd = 0;//1.077;

double error, lastError = 0;
double P, I, D;
double output;

// MPU6050 offsets
int accelXOffset = 0;
int accelYOffset = 0;
int accelZOffset = 0;
int gyroXOffset = 0;
int gyroYOffset = 0;
int gyroZOffset = 0;

// Tolerance range for the error
double errorTolerance = 1.0;

//bluetooth module code
#define BT_
SoftwareSerial bluetooth (10,9);

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

   byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    mpu.calcOffsets();
    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    Serial.println("Done!\n");

  attachInterrupt(digitalPinToInterrupt(2), encoderHandlerR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderHandlerR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), encoderHandlerL,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), encoderHandlerL, CHANGE);

  


  // Serial.println("Adafruit MPU6050 test!");

  // // Try to initialize!
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("MPU6050 Found!");

  // // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // // Serial.print("Accelerometer range set to: ");
  // // switch (mpu.getAccelerometerRange()) {
  // //   case MPU6050_RANGE_2_G:
  // //     Serial.println("+-2G");
  // //     break;
  // //   case MPU6050_RANGE_4_G:
  // //     Serial.println("+-4G");
  // //     break;
  // //   case MPU6050_RANGE_8_G:
  // //     Serial.println("+-8G");
  // //     break;
  // //   case MPU6050_RANGE_16_G:
  // //     Serial.println("+-16G");
  // //     break;
  // // }
  // // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // // Serial.print("Gyro range set to: ");
  // // switch (mpu.getGyroRange()) {
  // //   case MPU6050_RANGE_250_DEG:
  // //     Serial.println("+- 250 deg/s");
  // //     break;
  // //   case MPU6050_RANGE_500_DEG:
  // //     Serial.println("+- 500 deg/s");
  // //     break;
  // //   case MPU6050_RANGE_1000_DEG:
  // //     Serial.println("+- 1000 deg/s");
  // //     break;
  // //   case MPU6050_RANGE_2000_DEG:
  // //     Serial.println("+- 2000 deg/s");
  // //     break;
  // // }

  // // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // // Serial.print("Filter bandwidth set to: ");
  // // switch (mpu.getFilterBandwidth()) {
  // //   case MPU6050_BAND_260_HZ:
  // //     Serial.println("260 Hz");
  // //     break;
  // //   case MPU6050_BAND_184_HZ:
  // //     Serial.println("184 Hz");
  // //     break;
  // //   case MPU6050_BAND_94_HZ:
  // //     Serial.println("94 Hz");
  // //     break;
  // //   case MPU6050_BAND_44_HZ:
  // //     Serial.println("44 Hz");
  // //     break;
  // //   case MPU6050_BAND_21_HZ:
  // //     Serial.println("21 Hz");
  // //     break;
  // //   case MPU6050_BAND_10_HZ:
  // //     Serial.println("10 Hz");
  // //     break;
  // //   case MPU6050_BAND_5_HZ:
  // //     Serial.println("5 Hz");
  // //     break;
  // // }

  Serial.println("");
  delay(100);

  // Calibrate MPU6050 offsets

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
}

void loop() {
  mpu.update();

  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // // Apply offsets to accelerometer and gyroscope readings
  // int accelX = a.acceleration.x - accelXOffset;
  // int accelY = a.acceleration.y - accelYOffset;
  // int accelZ = a.acceleration.z - accelZOffset;
  // int gyroX = g.gyro.x - gyroXOffset;
  // int gyroY = g.gyro.y - gyroYOffset;
  // int gyroZ = g.gyro.z - gyroZOffset;

  // double angle = atan2(accelY, accelZ) * 180 / PI;
  // double gyroAngle = angle + (gyroX / 131.0) * 0.98;

  double gyroAngle = mpu.getAngleX();

  error = setpoint - gyroAngle;
  P = Kp * error;
  I += Ki * error;
  D = Kd * (error - lastError);
  output = P + I + D;
  lastError = error;

  // Check if the error is within the tolerance range
  if (fabs(error) < errorTolerance) {
    output = 0.0;  // Set the output to zero
    I = 0.0;      // Reset the integral term
  }

  if (output > 255.00) {
    output = 255.00;
  } else if (output < -255.00) {
    output = -255.00;
  }
  int motorSpeed = abs(output);  // Map the absolute output to the motor speed range
  
  if (output <= 0.00) {
    motorDirection = -1;
  } else {
    motorDirection = 1;
  }

  if (motorDirection == 1) {
    analogWrite(motor1Pin1, motorSpeed);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, motorSpeed);
    analogWrite(motor2Pin2, 0);
  } else if (motorDirection == -1) {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, motorSpeed);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, motorSpeed);
  }

  Serial.print("Angle:");
  Serial.println(gyroAngle);
  // delay(500);
  // Serial.print(","); 
  // Serial.print("Output:");
  // Serial.print(output);
  // Serial.print(","); 
  // Serial.print("Motor Speed:");
  // Serial.print(motorSpeed);
  // Serial.print(","); 
  // Serial.print("Motor Direction:");
  // Serial.println(motorDirection);
  // delay(20);  // Adjust delay as needed


  //the code to send the data via bluetooth module
  // if (bluetooth.available())
  // {
  //   int cmd =bluetooth.read();
  //   float input = (analogRead(A0)) / x;
  //   //Serial.println(input)
  // }
  // if (cmd == 1)
  // {
  //   if(input>5){bluetooth.write(closed);}
  //   else{bluetooth.write(opened);}
  // }
}
