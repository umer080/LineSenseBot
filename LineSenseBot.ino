#include <QTRSensors.h>
// Calibration ------------------------------------------------------------>>>6764
//Kp=0.01 , Kd=0, Base Speed = 70, Max Speed = 100 Speed turn 80----------->>>6774
//Kp=0.02 , Kd=2, Base Speed = 70, Max Speed = 100 Speed turn 80----------->>>6775

//BEST Kp 0.02 Kd 3 Max speed 100 Base speed 70 FInal---->>6773
#define Kp 0.02      // Proportional gain + experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 3         // Derivative gain
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define Ki 0         // Integral gain (not used)
#define MaxSpeed 100 // Maximum speed of the robot
#define BaseSpeed 70 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define speedturn 80 // Speed for turning

// Pin Definitions
#define rightMotor1 A1
#define rightMotor2 A2
#define rightMotorPWM 10
#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 11


// Global Variables
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int lastError = 0;

void setup() {
  // Initialize QTR sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(12);

  // Initialize motor pins
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  delay(3000); // Delay to allow setup
  Serial.begin(9600);
  Serial.println();

  // sensor_calibrate();

  for (int i = 0; i < 100; i++) { // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
    //comment this part out for automatic calibration
    if (i < 25 || i >= 75) {
      move(1, 70, 1); // Move right motor forward
      move(0, 70, 0); // Move left motor backward
    } else {
      move(1, 70, 0); // Move right motor backward
      move(0, 70, 1); // Move left motor forward
    }
    qtr.calibrate();
    delay(20);
  }

  wait(); // Wait for motors to stop
  delay(3000); // Delay 3secs to position the robot before entering the main loop
}


void loop() {
  // Read sensor values
  uint16_t position = qtr.readLineBlack(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  // Handle extreme cases where line is lost
  if (position > 6500) {
    move(1, speedturn, 1); // Turn right
    move(0, speedturn, 0);
    return;
  }
  if (position < 500) {
    move(1, speedturn, 0); // Turn left
    move(0, speedturn, 1);
    return;
  }

  // Calculate error and adjust motor speeds
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  // Ensure motor speeds are within limits
  if (rightMotorSpeed > MaxSpeed ) {rightMotorSpeed = MaxSpeed;} // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) {leftMotorSpeed = MaxSpeed;} // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)  {rightMotorSpeed = 0;}
  if (leftMotorSpeed < 0) {leftMotorSpeed = 0;}

  // Move motors
  move(1, rightMotorSpeed, 1); // Move right motor forward
  move(0, leftMotorSpeed, 1);  // Move left motor forward
}

// Function to stop motors
void wait() {
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}

// Function to control motor movement
void move(int motor, int speed, int direction) {
  boolean inPin1;
  boolean inPin2;

  // Determine pin states based on direction
  if(direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if(direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  // Set motor pins accordingly
  if(motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  } else if(motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
}

void sensor_calibrate() {

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
