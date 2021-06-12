/*****************************************************/
// prasoon@stratbeans.com
// https://github.com/stratbeans/sketch_arduino_obstacle_avoidance_car.git
/*****************************************************/

#include <AFMotor.h>
#include <Servo.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DEFINITIONS : Ultrasound sensor
int usTrigPin = 10;
int usEchoPin = 13;
#define US_OBSTACLE_CUTOFF_DISTANCE 60

// DEFINITIONS : DC Motor
AF_DCMotor motorBackRight(3);  // Channel 3
AF_DCMotor motorBackLeft(4);   // Channel 4
int speed = 150;               // Default forward, backward speed
int extraSpeed = 160;
#define TURN_TIME 500
#define BACKWARD_TIME 400

// DEFINITIONS : Servo Motor
int servoPin = 9;
Servo myservo;
#define SERVO_MOTION_DELAY 1000
const int ALIGN_CENTER = 90;
const int ALIGN_LEFT = 180;
const int ALIGN_RIGHT = 0;

// DEFINITIONS : Debugging
#define MOVE_FORWARD 1
#define MOVE_BACK 2
#define MOVE_STOP 3
#define TURN_LEFT 4
#define TURN_RIGHT 5

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// === Ultrasound sensor
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int checkUltrasoundDistance() {
  // clear trigger pin
  digitalWrite(usTrigPin, LOW);
  delayMicroseconds(2);

  // send 10 ms signal on trigger
  digitalWrite(usTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrigPin, LOW);

  // read echo pin
  long durationUS = pulseIn(usEchoPin, HIGH);

  // distance in cm
  int d = durationUS * 0.017;
  Serial.print("checkUltrasoundDistance : ");
  Serial.println(d);

  delay(100);
  return d;
}

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// === DC Motor
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void setupDCMotors() {

  // Set the speed of both wheel to equal
  motorSpeedMakeEqual();
}
void manageMotor(int mDirection) {
  Serial.print("manageMotor : ");
  Serial.println(mDirection);

  if (mDirection == RELEASE) {
    showMsg(MOVE_STOP);
  } else if (mDirection == FORWARD) {
    showMsg(MOVE_FORWARD);
  } else if (mDirection == BACKWARD) {
    showMsg(MOVE_BACK);
  }

  motorBackRight.run(mDirection);
  motorBackLeft.run(mDirection);
}

void motorSpeedMakeEqual() {
  Serial.println("motorSpeedMakeEqual");
  motorBackRight.setSpeed(speed);
  motorBackLeft.setSpeed(speed);
}

// backward oriented left
void motorTurnLeftSpeed() {
  Serial.println("motorTurnLeftSpeed");
  showMsg(TURN_LEFT);
  motorBackRight.setSpeed(0);
  motorBackLeft.setSpeed(0 + extraSpeed);
}

// backward oriented right
void motorTurnRightSpeed() {
  Serial.println("motorTurnRightSpeed");
  showMsg(TURN_RIGHT);
  motorBackRight.setSpeed(0 + extraSpeed);
  motorBackLeft.setSpeed(0);
}

void decideToReverseOrTurn(int sideDistance, char dChar)
{
    Serial.print("More space on ");
    Serial.println(dChar);

    if (sideDistance <= US_OBSTACLE_CUTOFF_DISTANCE) {
      Serial.print("Cant choose ");
      Serial.print(dChar);
      Serial.println(" as the distance is in collision zone, let me go back");

      // Sets same speed on both wheels for linear ( non turn ) motion
      motorSpeedMakeEqual();
      manageMotor(BACKWARD);
      delay(BACKWARD_TIME);
      return;
    } 

    // If code is here it means we have sufficient side distance
    Serial.print("More side distance on ");
    Serial.println(dChar);

    // Note: in the block below we are only setting the differntial
    // speed for making a turn. It would not set the car into motion
    if (dChar == 'L') {
        motorTurnLeftSpeed();
    }
    else {
        // dChar should be R
        motorTurnRightSpeed();
    }
    
    // Now move the car in reverse for certain time
    manageMotor(BACKWARD);
    delay(TURN_TIME);
}

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// === Servo motor
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setupServo() {
  // Connect the servo object to servoPin
  myservo.attach(servoPin);
  pinMode(usTrigPin, OUTPUT);
  pinMode(usEchoPin, INPUT);
  // Make the servo see in front
  positionServo(ALIGN_CENTER);
}

void positionServo(const int headDirection) {
  Serial.print("Position Servo ");
  Serial.println(headDirection);
  myservo.write(headDirection);
  delay(SERVO_MOTION_DELAY);
}

int rotateServoAndCheckDistance(const int servoDirection, char dChar)
{
  positionServo(servoDirection);
  int d = checkUltrasoundDistance();
  Serial.print(dChar);
  Serial.print(" Distance : ");
  Serial.println(d);
  delay(10);
  return d;
}

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// === DEBUGGING FUNCTIONS
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setupDebuggingPins() {
  // For debugging purposes
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  // Setup Dance of LED. This helped when Arduino restarts due to load issues
  // this dance indicates the process
  digitalWrite(A0, HIGH);
  delay(500);
  digitalWrite(A1, HIGH);
  delay(500);
  digitalWrite(A2, HIGH);
  delay(500);
  digitalWrite(A3, HIGH);
  delay(500);

  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void showMsg(int m) {
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);

  if (m == MOVE_FORWARD) {
    digitalWrite(A0, HIGH);
  } else if (m == MOVE_BACK) {
    digitalWrite(A1, HIGH);
  } else if (m == MOVE_STOP) {
    digitalWrite(A2, HIGH);
  } else if (m == TURN_LEFT) {
    digitalWrite(A3, HIGH);
  } else if (m == TURN_RIGHT) {
    digitalWrite(A0, HIGH);
    digitalWrite(A3, HIGH);
  }
}


void setup() {
  // 9600 baud rate
  Serial.begin(9600);

  // Welcome message
  Serial.println("Starting Robot Car !");

  setupDCMotors();
  setupServo();
  setupDebuggingPins();
}


void loop() {
  //return;
  // put your main code here, to run repeatedly:
  Serial.println("Looping...");

  int d = checkUltrasoundDistance();

  // if we are above cutoff distance, keep moving forward
  if (d >= US_OBSTACLE_CUTOFF_DISTANCE) {
    Serial.println("Zone clear");
    motorSpeedMakeEqual();
    manageMotor(FORWARD);
    return;
  }

  // If code reaches here, it means we are in collision zone
  Serial.print("Collision zone ");
  Serial.print(d);
  Serial.print(", ");
  Serial.println(US_OBSTACLE_CUTOFF_DISTANCE);

  // Stop the motor to avoid forward moving accident and wait
  manageMotor(RELEASE);

  // move Servo left and measure distance
  int leftDistance = rotateServoAndCheckDistance(ALIGN_LEFT, 'L');

  // move Servo right and measure distance
  int rightDistance = rotateServoAndCheckDistance(ALIGN_RIGHT, 'R');

  // Now revert back servo to center
  positionServo(ALIGN_CENTER);

  // Since car has seen left and right distance, let the function below
  // take decision to turn or go back reverse
  decideToReverseOrTurn(leftDistance, leftDistance > rightDistance ? 'L' : 'R');
}
