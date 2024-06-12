#include <ESP32Servo.h>
#include <BMI088.h>
#include <NewPing.h>

Servo servo;  // create servo object to control a servo

// Gyro sensor
#define GYRO_SAMPLE_SIZE 1
#define DRIFT_TEST_TIME 10
#define INT_PIN A6

// Gyro sensor
Bmi088Gyro gyro(Wire,0x69);
Bmi088Accel accel(Wire,0x19);

// Gyro sensor
double gyro_last_read_time = 0;
double drifts_x = 0, drifts_y = 0, drifts_z = 0;
bool gyro_flag, accel_flag;
double gx, gy, gz;
long cnt1 = 0;

/// Gyro functions

void gyro_drdy()
{
  gyro_flag = true;
}

void accel_drdy()
{
  accel_flag = true;
}

void gyro_setup(bool debug) {
  int status = accel.begin();
  status = accel.setOdr(Bmi088Accel::ODR_200HZ_BW_80HZ);
  status = accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
  status = accel.mapDrdyInt1(true);


  status = gyro.begin();

  status = gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  status = gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
  status = gyro.mapDrdyInt3(true);

  pinMode(INT_PIN,INPUT);
  attachInterrupt(INT_PIN,gyro_drdy,RISING);  


  if(status < 0) {
    if(debug) {
      Serial.print("BMI Initialization Error!  error: ");
      Serial.println(status);
    }
  }
  else  {
    // Gyro drift calculation
    if(debug) Serial.println("Starting gyro drift calculation...");

    gx = 0;
    gy = 0;
    gz = 0;

    gyro_last_read_time = millis();

    double start_time = millis();
    while(millis() - start_time < DRIFT_TEST_TIME * 1000) {
      gyro.readSensor();  
      double read_time = millis();
      gx += (gyro.getGyroX_rads() * (read_time - gyro_last_read_time) * 0.001);
      gy += (gyro.getGyroY_rads() * (read_time - gyro_last_read_time) * 0.001);
      gz += (gyro.getGyroZ_rads() * (read_time - gyro_last_read_time) * 0.001);

      gyro_last_read_time = read_time;
    }

    drifts_x = gx / DRIFT_TEST_TIME;
    drifts_y = gy / DRIFT_TEST_TIME;
    drifts_z = gz / DRIFT_TEST_TIME;

    if(debug) Serial.print("Drift test done!\nx: ");
    if(debug) Serial.print(drifts_x, 6);
    if(debug) Serial.print("   y: ");
    if(debug) Serial.print(drifts_y, 6);
    if(debug) Serial.print("   z: ");
    if(debug) Serial.println(drifts_z, 6);
  }
  // Gyro value reset
  gx = 0;
  gy = 0;
  gz = 0;

  gyro_last_read_time = millis();
}

void read_gyro(bool debug) {
  //delta_start = millis();
  if(gyro_flag) {
    gyro_flag = false;
    cnt1++;
    gyro.readSensor();   
    double read_time = millis();

    gx += ((gyro.getGyroX_rads() - drifts_x) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
    gy += ((gyro.getGyroY_rads() - drifts_y) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;
    gz += ((gyro.getGyroZ_rads() - drifts_z) * (read_time - gyro_last_read_time) * 0.001) * 180.0 / PI;

    gyro_last_read_time = read_time;

    //delta_gyro = millis() - delta_start;

    if(debug) {
      Serial.print("Gyro: gx: ");
      Serial.print(gx);
      Serial.print(" gy: ");
      Serial.print(gy);
      Serial.print(" gz: ");
      Serial.println(gz);
    }
  }
}

const int trigFront = 2;
const int echoFront = 3;
const int trigDR = 10;
const int echoDR = 11;
const int MaxSensorDist = 150;
const int BUTTON_PIN = 12;
const int PWMA = 6;
const int AIN1 = 7;
const int AIN2 = 8;
const int servoPin = 9;

NewPing sonarFR(trigFront, echoFront, MaxSensorDist);
NewPing sonarDR(trigDR, echoDR, MaxSensorDist);

// LEDC PWM channels and frequency
const int freq = 5000;
const int pwmChannel = 1;
const int resolution = 10;

long duration;
int distanceFront, distanceST, distanceDR;

int HighServoLimit;
int LowServoLimit;
int MiddleServoDeg;
void setup() {
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigDR, OUTPUT);
  pinMode(echoDR, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  //while (!Serial) ;

  // Enable the motor driver
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Configure LEDC PWM for motor speed control
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWMA, pwmChannel);
  ledcWrite(pwmChannel, 0);

  servo.attach(servoPin, 544, 1850);  // attaches the servo on pin 9 to the servo object
  delay(1000);
  servo.write(110);
  delay(1000);
  //Serial.println(servo.read());
  HighServoLimit = servo.read();
  servo.write(70);
  delay(1000);
  //Serial.println(servo.read());
  LowServoLimit = servo.read();

  //MiddleServoDeg = (HighServoLimit+LowServoLimit)/2;
  MiddleServoDeg = 87;
  //Serial.println(MiddleServoDeg);
  servo.write(MiddleServoDeg);
  delay(2000);
  gyro_setup(true);
  digitalWrite(LED_BUILTIN, HIGH);
  waitForButtonPress();
}

void waitForButtonPress() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configure button pin as input with internal pull-up resistor

  Serial.println("Waiting for button press...");
  while (digitalRead(BUTTON_PIN) == HIGH) {
    // Wait here until the button is pressed
  }
  Serial.println("Button pressed, starting loop...");
}


int wasmotorstopped = 0;
int error = 0;
int degServo = 0;
int currMiddleZ = 0;
int distminFront = 45;
int turns = 0;
int maxstreak = 1;
int streak = 0;
int turnDir = 0;
int timer;
int timerlenght;

void loop()
{
  distanceFront = distminFront+10;
  //distanceDR = getDistance(trigDR, echoDR);
  distanceDR = sonarDR.ping_cm();
  //delay(100);
  while(distanceFront > distminFront || distanceFront<=10)
  {
    read_gyro(false);
    //delay(100);
    if(gz>=currMiddleZ)
    {
      error = gz-currMiddleZ;
      degServo = MiddleServoDeg-error;
    }
    else if(gz<currMiddleZ)
    {
      error = currMiddleZ-gz;
      degServo = MiddleServoDeg+error;
    }
    Serial.print(degServo);
    Serial.print("  ");
    Serial.print(distanceDR);
    Serial.print("  ");
    Serial.println(distanceFront);
    if(degServo>HighServoLimit)
      degServo=HighServoLimit;
    if(degServo<LowServoLimit)
      degServo=LowServoLimit;
    servo.write(degServo);
    moveForward(500);

    if(currMiddleZ-5<=gz && gz<=currMiddleZ+5)
    {
      streak++;
      if(streak==maxstreak)
      {
        //distanceFront = getDistance(trigFront, echoFront);
        distanceFront = sonarFR.ping_cm();
        //delay(100);
        streak = 0;
      }
      else
        distanceFront = distminFront+10;
    }
    else
      streak = 0;
    //distanceDR = getDistance(trigDR, echoDR);
    //distanceDR = sonarDR.ping_cm();
    //delay(100);
  }
  stopMotor();

  if(turns!=12)
  {
    Serial.print("Turning... ");
    Serial.println(distanceFront);
    distanceDR = sonarDR.ping_cm();
    //delay(1000);
    Serial.println(distanceDR);
    //delay(1000);
    if(turns==0)
    {
      if(distanceDR!=0 && distanceDR<=90)
        turnDir = 1;
        Serial.print("DistDR: ");
        Serial.println(distanceDR);
    }
    if(turnDir==0)
    {
      currMiddleZ += 90;
      degServo = HighServoLimit;
    }
    else if(turnDir==1)
    {
      currMiddleZ -= 90;
      degServo = LowServoLimit;
    }
    servo.write(degServo);
    //delay(1000);
    read_gyro(true);
    //delay(100);
    if(turnDir==0)
    {
      while(gz+2<currMiddleZ)
      {
        moveForward(400);
        read_gyro(true);
       // delay(100);
      }
    }
    else if(turnDir==1)
    {
      while(gz-2>currMiddleZ)
      {
        moveForward(400);
        read_gyro(true);
        //delay(100);
      }
    }
    stopMotor();
    Serial.println("Turn Complete!");
    degServo = MiddleServoDeg;
    servo.write(degServo);
    turns++;
    if(turns==12)
    {
      distminFront = 160;
      timer = millis();
      while(millis()-timer<=2000)
      {
        read_gyro(false);
        //delay(100);
        if(gz>=currMiddleZ)
        {
          error = gz-currMiddleZ;
          degServo = MiddleServoDeg-error;
        }
        else if(gz<currMiddleZ)
        {
          error = currMiddleZ-gz;
          degServo = MiddleServoDeg+error;
        }
        if(degServo>HighServoLimit)
          degServo=HighServoLimit;
        if(degServo<LowServoLimit)
          degServo=LowServoLimit;
        servo.write(degServo);
        moveForward(500);
      }
      stopMotor();
    }
    //delay(1000);
  }
  if(turns==12)
  {
    while(true)
    {

    }
  }
  //read_gyro(true);
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
