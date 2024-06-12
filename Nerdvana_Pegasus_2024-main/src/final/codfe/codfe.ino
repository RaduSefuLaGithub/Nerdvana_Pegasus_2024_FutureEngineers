#include <ESP32Servo.h>
#include <BMI088.h>

Servo servo;  // create servo object to control a servo

Bmi088Gyro gyro(Wire,0x69);

const int trigFront = 2;
const int echoFront = 3;
////const int trigSide = 4;
const int echoSide = 5;
const int PWMA = 6;
const int AIN1 = 7;
const int AIN2 = 8;
const int servoPin = 9;

// LEDC PWM channels and frequency
const int freq = 5000;
const int pwmChannel = 1;
const int resolution = 10;

long duration;
int distanceFront, distanceSide;

void setup() {
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  //pinMode(trigSide, OUTPUT);
  //pinMode(echoSide, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  Serial.begin(9600);
  while (!Serial) ;

  // Enable the motor driver
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Configure LEDC PWM for motor speed control
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWMA, pwmChannel);
  ledcWrite(pwmChannel, 0);

  servo.attach(servoPin, 544, 1850);  // attaches the servo on pin 9 to the servo object
  servo.write(90);

  /*int status;
  status = gyro.begin();
  if(status>=0)
    Serial.println("Gyro Is set!");
  else if(status<0)
    Serial.println("Gyro Failled Setup!");*/

  //Serial.println("1");
  //Serial.println(status);

  //gyro.mapDrdyInt3(true);
}


float gx, gy, gz;
int wasmotorstopped = 0;

void loop() {
  //Serial.println("2");
  distanceFront = getDistance(trigFront, echoFront);
  Serial.print("DistanceF: ");
  Serial.println(distanceFront);
  //distanceSide = getDistance(trigSide, echoSide);
  //Serial.print("DistanceS: ");
  //Serial.println(distanceSide);

  if (distanceFront < 50) // if an obstacle is detected in front
  {
      stopMotor();             // stop motor
  }
  else
  {
    moveForward(1023);
  }
    /*int val;
    for(val=0; val<25; val++)
      servo.write(val);
    for(val=25; val>0; val--)
      servo.write(val);*/

      /*gyro.readSensor();

      gx = gyro.getGyroX_rads();
      gy = gyro.getGyroY_rads();
      gz = gyro.getGyroZ_rads();

      Serial.print("gx:");
      Serial.print(gx);

      Serial.print(", gy:");
      Serial.print(gy);

      Serial.print(", gz:");
      Serial.println(gz);
      delay(1000);*/
      digitalWrite(13, HIGH);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void moveForward(int motorSpeed) {
  if (motorSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    motorSpeed *= -1;
  }
  ledcWrite(pwmChannel, motorSpeed);
}

void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannel, 0);
}
