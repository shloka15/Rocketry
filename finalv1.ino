#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "PID_v1.h"
#include <LoRa.h>
#include <Wire.h>    //Include wire library 
#include "MPU6050_light.h"  //Include library for MPU communication  //Library for LCD Display
#include <Adafruit_BMP280.h>

Servo ServoX1, ServoY1, parachuteServo;
MPU6050 mpu(Wire);   //Create object mpu
    //Define LCD address and dimension
Adafruit_BMP280 bmp;

const int servoPin = 9;
const float seaLevelPressure = 1013.25;
float maxAltitude = 0;
bool apogeeReached = false;
int i;
float h;
double setpoint;
double ixangle = 0.0, oxangle = 0.0;
double iyangle = 0.0, oyangle = 0.0;
double kp = 2, ki = 1.15, kd = 3;
double altitude,verticalAcceleration;
#define PIN_INPUT 0
#define PIN_OUTPUT 3
PID myPIDX(&ixangle, &oxangle, &setpoint, kp, ki, kd, DIRECT);
PID myPIDY(&iyangle, &oyangle, &setpoint, kp, ki, kd, DIRECT);


unsigned long timer = 0;    

void setup() {
  ServoX1.attach(2);
  ServoY1.attach(4);
  Serial.begin(9600);    //Start serial communication
   
  setpoint = 0.00;
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);

  mpu.begin();     
  Serial.print(F("MPU6050 status: "));
  Serial.println(F("Calculating offsets, do not move MPU6050"));   
  delay(1000);
  mpu.calcGyroOffsets();     //Calibrate gyroscope
  Serial.println("Done!\n");
  // bmp.begin(BMP280_ULTRALOWPOWER);

  if (!bmp.begin()) 
  {
    Serial.println("BMP280 initialization failed!");
    while (1);
  }
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                Adafruit_BMP280::SAMPLING_X2,
                Adafruit_BMP280::SAMPLING_X16,
                Adafruit_BMP280::FILTER_X16,
                Adafruit_BMP280::STANDBY_MS_1);
  
  
  parachuteServo.attach(servoPin);
  parachuteServo.write(0);

  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  } else if (verticalAcceleration < 1.1 && verticalAcceleration >0.9) {
    // Apogee detected
    apogeeReached = true;
    deployParachute();
  }

  if (!LoRa.begin(915E6))
   {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  Serial.println("");
  delay(100);

}
void loop() {
  mpu.update();    //Get values from MPU
  pinMode(LED_BUILTIN, OUTPUT);
  { // print data every 100ms
    // Serial.print("AngleX:");
    // Serial.print(mpu.getAngleX());  
    // Serial.print(",");
    // Serial.print("AngleZ:");
    // Serial.print(mpu.getAngleZ()); 
    // Serial.print("\n");  
    ixangle=KALMAN(mpu.getAngleX(), 1);
    iyangle=KALMAN(mpu.getAngleZ(), 2);
    myPIDX.Compute();
    myPIDY.Compute();
    Serial.print("diffx:");
    Serial.print(abs(ixangle-mpu.getAngleX()));
    Serial.print(",diffz:");
    Serial.println(abs(iyangle-mpu.getAngleZ()));
    ServoX1.write(oxangle);  
    ServoY1.write(oyangle);
  }
}

double KALMAN(double U, int axis)
{
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P1 = 0;
  static double U_hat1 = 0;
  static double K1 = 0.2;
  static double P2 = 0;
  static double U_hat2 = 0;
  static double K2 = 0.2;
  if (axis == 1)
  {
  K1 = P1*H/(H*P1*H+R);
  U_hat1 += K1*(U-H*U_hat1);
  P1 = (1-K1*H)*P1+Q;
  // Serial.print("kalmanx:");
  // Serial.print(U_hat1);
  // Serial.print(",");
  return U_hat1;
  }
  else if(axis==2)
  {
  K2 = P2*H/(H*P2*H+R);
  U_hat2 += K2*(U-H*U_hat2);
  P2 = (1-K2*H)*P2+Q;
  // Serial.print("kalmanZ:");
  // Serial.print(U_hat2);
  // Serial.print(",");
  return U_hat2;

  }
return 0;
}


void deployParachute() {
  Serial.println("Apogee reached! Deploying parachute...");
  parachuteServo.write(180);  // Rotate servo 180 degrees
  
  // Send apogee event via LoRa
  LoRa.beginPacket();
  LoRa.print("APOGEE");
  LoRa.endPacket();
}
