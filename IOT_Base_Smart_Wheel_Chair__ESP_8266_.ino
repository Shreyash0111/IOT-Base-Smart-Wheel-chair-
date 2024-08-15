/*
Led pin = D0
GyroScope Pin (scl) = D1
GyroScope Pin (sda) = D2
Motor Driver IN1  = D3
Motor Driver IN2  = D4
Motor Driver IN3  = D5
Motor Driver IN4  = D6
Rain sensor pin   = D7
Servo Motor pin   = D8
LDR Module  pin   = A0
 */


#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

char auth[] = "n88XbPWEHvPSsSeJFbT6SGy1sJLIBg5u"; // Blynk authentication token
char ssid[] = "ABCD"; // WiFi SSID
char pass[] = "123456789"; // WiFi password

MPU6050 mpu;
Servo servoMotor;
#define IN1 D3
#define IN2 D4
#define IN3 D5
#define IN4 D6
const int ldrPin = 1;
const int ledPin = D0;
const int rainSensorPin = D7; 
const int servoPin = D8;  
bool forward = 0;
bool backward = 0;
bool left = 0;
bool right = 0;
int Speed;

void setup() {
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  
  Wire.begin();
  mpu.initialize();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(rainSensorPin, INPUT);
  pinMode(servoPin, OUTPUT);

  servoMotor.attach(servoPin);
  servoMotor.write(0); // Set servo initially to 0 degrees
}
BLYNK_WRITE(V0) {
  forward = param.asInt();
}

BLYNK_WRITE(V1) {
  backward = param.asInt();
}

BLYNK_WRITE(V2) {
  left = param.asInt();
}

BLYNK_WRITE(V3) {
  right = param.asInt();
}

void smartcar() {
  if (forward == 1) {
    Forward();
    Serial.println("Forward");
  }
 else if (backward == 1) {
    Backward();
    Serial.println("Backward");
  } else if (left == 1) {
    Left();
    Serial.println("Left");
  } else if (right == 1) {
    Right();
    Serial.println("Right");
  } 
  else if (forward == 0 && backward == 0 && left == 0 && right == 0  ) {
    Stop();
    Serial.println("Stop");
  }
}

void loop() {
  Blynk.run();


  
int sensorValue = digitalRead(ldrPin);
 // Serial.print("LDR Sensor Value: ");
 //Serial.println(sensorValue);
 if (sensorValue == 1 ) { 
    digitalWrite(ledPin, HIGH); 
    } else {
    digitalWrite(ledPin, LOW);
  } 

 
   int rainStatus = digitalRead(rainSensorPin);
 if (rainStatus == HIGH) {
    servoMotor.write(90);
     Blynk.logEvent("alert", "There is raining");
    } 
   else {
    servoMotor.write(0);
    } 

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate total acceleration magnitude
  float totalAcc = sqrt(ax*ax + ay*ay + az*az);

  // Print gyro data to serial monitor
  Serial.print("Gyro X: ");
  Serial.print(gx);
  Serial.print(" | Gyro Y: ");
  Serial.print(gy);
  Serial.print(" | Gyro Z: ");
  Serial.println(gz);
Serial.println(totalAcc);
  // If total acceleration falls below a threshold, notify via Blynk
  if (totalAcc < 17000) {
   Blynk.logEvent("wheel_chair", "there is something wrong");
    delay(100); // Avoid sending multiple notifications for the same fall
  }

  delay(100); // Adjust delay as needed
}
void Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void Backward() {
 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void Left() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void Right() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
}
