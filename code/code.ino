#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1_bc.h>
#include <VL53L0X.h> // LiDAR library

// IR sensor pins
#define IR_LEFT 34
#define IR_RIGHT 35

// LiDAR
VL53L0X lidar;
// MPU6050
MPU6050 mpu(Wire);

float angle = 0.0, rawAngle = 0.0,prevAngle = 0.0;

// Motor Pins
#define motor1_in1 12
#define motor1_in2 14
#define motor2_in3 27
#define motor2_in4 26
#define ENA 13
#define ENB 25

// Encoder Pins
#define encoder1_a 4
#define ENCA_B 18
#define encoder2_a 19
#define ENCB_B 23

// Movement Parameters
#define wheel_circumference 150.0
#define pulses_per_revolution 210
#define cell_distance 180.0
#define pulses_per_cell (int)((cell_distance / wheel_circumference) * pulses_per_revolution)

// Variables
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;
volatile int target_count = 0;

// MPU6050 Variables
float currentAngle = 0.0;
float targetAngle = 0.0;

int maxSpeed = 255;
int minSpeed = 160;
int leftSpeedVal;
int rightSpeedVal;

// Function Prototypes
void IRAM_ATTR encoder1_isr();
void IRAM_ATTR encoder2_isr();
void moveForwardOneCell();
void turnRight();
void stopMotors();
void forward();
void driving();
void controlSpeed();
void getAngel();
void setupIR();
void setupLiDAR();
bool readIRLeft();
bool readIRRight();
int readLiDAR();

void setup() {
  // Serial Monitor
  Serial.begin(9600);
  
  // Motor and Encoder Pin Setup
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(encoder1_a, INPUT_PULLUP);
  pinMode(encoder2_a, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(encoder1_a), encoder1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_a), encoder2_isr, RISING);
  
  
  Wire.begin(21, 22);  // SDA, SCL

  // Initialize MPU6050
  mpu.begin();
  mpu.calcOffsets();
  mpu.update();

  // Initialize sensors
  setupIR();
  setupLiDAR();

  // Reset Encoder Counts
  encoder1_count = 0;
  encoder2_count = 0;
  target_count = pulses_per_cell;
  delay(1000); 
}

void loop() {

  //read lidar - front wall
  int distance = readLiDAR();
  if (distance < 20){   //front wall within 20 cm
    stopMotors();
    delay(500);
    bool irLeft = readIRLeft();
    bool irRight = readIRRight();
    if (!irLeft && irRight) {     //there is a wall in the left, right is open
      Serial.print("Left IR activated,,, turning right ");
      stopMotors();
      delay(500);
      turnRight();
      move();
    }
    else if (!irRight && irLeft) {     //there is a wall in the right, left is open
      Serial.print("Right IR activated,,, turning left ");
      stopMotors();
      delay(500);
      turnLeft();
      move();
    }
    else{ //stuck
      Serial.print("stuck");
    }
  }
  else{
    move();
  }
  delay(700);
  
/*
  bool irLeft = readIRLeft();
  bool irRight = readIRRight();
  if (!irLeft) {     //there is a wall in the left
    Serial.print("Left IR activated,,, turning right ");
    stopMotors();
    delay(500);
    turnRight();
    move();
  }
  
  else if (!irRight) {     //there is a wall in the left
    Serial.print("Right IR activated,,, turning left ");
    stopMotors();
    delay(500);
    turnLeft();
    move();
  }
  else {
    move();
  }
  delay(700);
*/
}
  

const float correctionFactor = 2.3; // Adjust this value as needed
const int baseSpeed = 220; // Adjust the base motor speed as needed

void turnRight() {
  
  
  rawAngle = mpu.getAngleZ();
  angle = abs(rawAngle);

  float targetAngle = prevAngle + 70.0; // Calculate the target angle
  
  while (angle < targetAngle) {
    mpu.update();
    rawAngle = mpu.getAngleZ();
    angle = abs(rawAngle);

    Serial.print("Angle: ");
    Serial.println(angle);

    // Calculate the correction factor
    float correction = (targetAngle - angle) * correctionFactor;
    

    // Turn Right
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, LOW);
    digitalWrite(motor2_in4, HIGH);
    analogWrite(ENA, max((int)(baseSpeed + correction),100));
    analogWrite(ENB, max((int)(baseSpeed - correction),100));

  }
  
  stopMotors();
  
  prevAngle = angle; // Update the previous angle
}



void turnLeft() {
  rawAngle = mpu.getAngleZ();
  angle = rawAngle;  // Use signed value

  float targetAngle = prevAngle - 80.0; // Left turn target
  
  while (angle > targetAngle + 2.0) {  // Ensuring it completes the turn
    mpu.update();
    rawAngle = mpu.getAngleZ();
    angle = rawAngle; // Update angle

    Serial.print("Current Angle: ");
    Serial.print(angle);
    Serial.print(" | Target Angle: ");
    Serial.println(targetAngle);

    // Ensure a bigger correction difference
    float correction = (targetAngle - angle) * correctionFactor;
    
    

    // Apply motor directions for left turn
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, HIGH);
    digitalWrite(motor2_in3, HIGH);
    digitalWrite(motor2_in4, LOW);
    analogWrite(ENA,  max((int)(baseSpeed - correction),100));
    analogWrite(ENB,  max((int)(baseSpeed + correction),100));
  }
  
  stopMotors();
  prevAngle = targetAngle; // Update the previous angle
}



void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void IRAM_ATTR encoder1_isr() {
  encoder1_count++;
}

void IRAM_ATTR encoder2_isr() {
  encoder2_count++;
}
///////////////////////////////////////////////////

void move() {
  // Reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;

  // Update the target angle
  mpu.update();
  targetAngle = mpu.getAngleZ();

  while ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    mpu.update();
    float currentAngle = mpu.getAngleZ();
   // Serial.print("Target "); 
   // Serial.println(targetAngle);
   // Serial.print("Current ");
   // Serial.println(currentAngle);
    adjustMotorSpeeds(currentAngle, targetAngle);
   
    // Set motor directions to move forward
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, HIGH);
    digitalWrite(motor2_in4, LOW);

    // Apply speeds to motors
    //analogWrite(ENA, baseSpeed);
    //alogWrite(ENB, baseSpeed);
  }

  stopMotors();
  //Serial.println("Reached target distance.");
}

// PID parameters
float Kp = 4.0;  // Proportional gain
float Ki = -0.8;  // Integral gain
float Kd = 1.0;  // Derivative gain

// PID variables
float integral = 0;
float lastError = 0;
float derivative = 0;

void adjustMotorSpeeds(float currentAngle, float targetAngle) {
  float error = targetAngle - currentAngle;
  integral += error;  // Accumulate the error over time
  derivative = error - lastError;  // Calculate the change in error

  int speedAdjustment = (int)(Kp * error + Ki * integral + Kd * derivative);  // PID formula

  int base = 60; 
  int leftSpeed = base - speedAdjustment;
  int rightSpeed = base + speedAdjustment;
  
  // Constrain speeds toc allowable range
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
   
  // Apply adjusted speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
/*
  Serial.print("left "); 
  Serial.println(leftSpeed);
  Serial.print("Right ");
  Serial.println(rightSpeed);
*/
  // Update lastError for the next cycle
  lastError = error;

}


// Function to initialize IR sensors
void setupIR() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

// Function to initialize LiDAR sensor
void setupLiDAR() {
  lidar.setTimeout(500);
  if (!lidar.init()) {
    Serial.println("Failed to detect and initialize LiDAR sensor");
    while (1); // Stop execution if LiDAR fails
  }
}

// Function to read the left IR sensor
//active low
bool readIRLeft() {
  return digitalRead(IR_LEFT); // 0 = obstacle detected, 1 = clear
}

// Function to read the right IR sensor
//active low
bool readIRRight() {
  return digitalRead(IR_RIGHT); // 0 = obstacle detected, 1 = clear
}
bool readIRRight2() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(IR_LEFT);
  if (currentState != lastState) {
    delay(50); // Debounce delay
    lastState = currentState;
  }
  return lastState;
}


// Function to read LiDAR distance in centimeters
int readLiDAR() {
  int distance = lidar.readRangeSingleMillimeters() / 10; // Convert mm to cm
  if (lidar.timeoutOccurred()) {
    Serial.println("LiDAR Timeout");
    return -1; // Return -1 if timeout occurs
  }
  return distance;
}