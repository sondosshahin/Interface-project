// Testing with floodFill

#include <Wire.h>
#include <MPU6050_light.h>
#include <VL53L0X.h> // LiDAR library

// IR sensor pins
#define IR_LEFT 35
#define IR_RIGHT 34

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
#define ENA 13 //Left 
#define ENB 25 // Right 

// Encoder Pins
#define encoder1_a 4
#define ENCA_B 18
#define encoder2_a 19
#define ENCB_B 23

#define LED_PIN 15

// Movement Parameters
#define wheel_circumference 135.0
#define pulses_per_revolution 210
#define cell_distance 170.0
#define pulses_per_cell (int)((cell_distance / wheel_circumference) * pulses_per_revolution)

// Direction Variables
#define FORWARD   0
#define RIGHT     1
#define BACKWARD  2
#define LEFT      3

// Variables
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;
volatile int target_count = 0;

// MPU6050 Variables
float currentAngle = 0.0;
float targetAngle = 0.0;

int maxSpeed = 255;
int minSpeed = 100;
int baseSpeed = 60; 

int leftSpeedVal;
int rightSpeedVal;

// Algorithim Variable 
const int mazeSize= 8;
int currentOrientation = 0;

struct Position {
  int x;
  int y;
};

// Initialize the robot's current position
Position currentPosition = {mazeSize - 1, 0}; // Starting at bottom-left corner

int floodArray[mazeSize][mazeSize]; 
// Assume wallArray to track the in-between walls
int wallArray[mazeSize-1][mazeSize-1];
// check visited cells
bool visitedArray[mazeSize][mazeSize] = {false}; // Initialize all cells as unvisited

bool isLearning = true;

// Function Prototypes
void IRAM_ATTR encoder1_isr();
void IRAM_ATTR encoder2_isr();
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
void floodFillSetUp();
void setCurrentPosition();

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
  pinMode(encoder2_a, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);
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

  // SetUp Algorithim
  floodFillSetUp();
  currentOrientation = FORWARD;

  setCurrentPosition();

  isLearning = true;
}

void loop() {

  if (isLearning){

    int decision = makeDecision();

    if (decision == RIGHT ){
        stopMotors();
        delay(500);
        turnRight();
        move();
    } else if (decision == LEFT ){
        stopMotors();
        delay(500);
        turnLeft();
        move();
    } else if (decision == BACKWARD){
        stopMotors();
        move_backward();
        stopMotors();
        delay(2000);
    } else{ // FORWARD
        move();
        delay(500);
    }

  } else { // walk optimal phase : maze already stored

  }


  /*
  if (!centerReached()){

    //read lidar - front wall
    int distance = readLiDAR();
    Serial.println(distance);
    
    if (distance < 20){   //front wall within 20 cm
      digitalWrite(LED_PIN, HIGH);  // Turn LED ON 

      stopMotors();
      delay(500);
      bool irLeft = readIRLeft();
      bool irRight = readIRRight();

      if (!irLeft && irRight) { //there is a wall in the left, right is open
        Serial.print("Left IR activated,,, turning right ");
        stopMotors();
        delay(500);
        turnRight();
        move();
      }
      else if (irLeft && irRight) { //there is a wall in the left, right is open
        Serial.print("Left IR activated,,, turning right ");
        stopMotors();
        delay(500);
        turnRight();
        move();
      }
      else if (!irRight && irLeft) { //there is a wall in the right, left is open
        Serial.print("Right IR activated,,, turning left ");
        stopMotors();
        delay(500);
        turnLeft();
        move();
      }
      else{

        //stuck
        Serial.println("stuck");
        move_backward();
        stopMotors();
        delay(2000);
        bool irLeft = readIRLeft();
        bool irRight = readIRRight();

        if (!irLeft && irRight) {     //there is a wall in the left, right is open
          Serial.print("Left IR activated,,, turning right ");
          stopMotors();
          delay(500);
          turnRight();
          move();
        }
        else if (irLeft && irRight) {     //the left is open, right is open
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
      }
    
    }
    else{
      digitalWrite(LED_PIN, LOW);  // Turn LED ON 
      move();
    }

    updatePosition();
    delay(700);
  }
  else {

    // center reached stop 
    stopMotors();
    while(1);
  }*/

 
}


const float correctionFactor = 0.25; // Adjust this value as needed

void turnRight() {
  

  currentOrientation = (currentOrientation + 1) % 4;

  stopMotors();
  mpu.update();  
  rawAngle = mpu.getAngleZ();
  //angle = abs(rawAngle); 
  angle = rawAngle;
  prevAngle = angle;
  targetAngle = prevAngle + 85.0; // Calculate the target angle  was 50
  //targetAngle = 70; 

   Serial.print("Angle before");
   Serial.println(prevAngle); 

  while (angle < targetAngle - 2.0) {
    mpu.update();
    rawAngle = mpu.getAngleZ();
    angle = rawAngle;

    Serial.print("Angle: ");
    Serial.println(angle);

    // Calculate the correction factor
    float correction = (targetAngle - angle) * correctionFactor;
    

    // Turn Right
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, HIGH);
    digitalWrite(motor2_in4, LOW);
    //analogWrite(ENA, (int)(80 + correction));
    //analogWrite(ENB, (int)(80 - correction) );
    analogWrite(ENA, 60);
    analogWrite(ENB, 60);
    //analogWrite(ENA, max((int)(baseSpeed + correction),100));
    //analogWrite(ENB, max((int)(baseSpeed - correction),100));

  }
   Serial.print("target");
   Serial.println(targetAngle); 
   Serial.print("Angle");
   Serial.println(angle); 
  stopMotors();
  prevAngle = angle; // Update the previous angle
  delay(500); // Give time before turning
  
}



void turnLeft() {

  currentOrientation = (currentOrientation + 3) % 4;

  stopMotors();
  mpu.update();
  rawAngle = mpu.getAngleZ();
  angle = rawAngle;  // Use signed value
  prevAngle = angle; 
  targetAngle = prevAngle - 83.0; // Left turn target
   Serial.print("Angle before");
   Serial.println(prevAngle); 
  while (angle > targetAngle + 2.0) {  // Ensuring it completes the turn
    mpu.update();
    rawAngle = mpu.getAngleZ();
    angle = rawAngle; // Update angle
/*
    Serial.print("Current Angle: ");
    Serial.print(angle);
    Serial.print(" | Target Angle: ");
    Serial.println(targetAngle);
    */

    // Ensure a bigger correction difference
    float correction = (targetAngle - angle) * correctionFactor;
    
  
    // Apply motor directions for left turn
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, HIGH);
    //digitalWrite(motor1_in2, LOW);

    digitalWrite(motor2_in3, LOW);
     digitalWrite(motor2_in4, HIGH);
     //digitalWrite(motor2_in4, LOW);

    //analogWrite(ENA,  (int)(80 - correction));
    //analogWrite(ENB,  (int)(80 + correction));

    analogWrite(ENA, 60);
    analogWrite(ENB, 80);
   
    
  }
   Serial.print("target");
   Serial.println(targetAngle); 
   Serial.print("Angle");
   Serial.println(angle); 


  stopMotors();
  prevAngle = angle; // Update the previous angle
  delay(500); // Give time before turning
  
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

//////////////////////////////////////////////////////////////////////////////////////////////////////

void setCurrentPosition(){ // upgrade to use bluetooth
  // the default is starting at bottom-left corner
  Position currentPosition = {mazeSize - 1, 0}; 

}

void floodFillSetUp(){
  // set up manhattan distance for each cell intially 
  for (int i=0; i< mazeSize/2; i++){
      for (int j=0; j< mazeSize/2; j++){
        // solve for one quatrer and set the rest 
        int dist = (mazeSize / 2 - i - 1) + (mazeSize / 2 - j- 1);

        floodArray[i][j] = dist;
        floodArray[mazeSize - 1 - i][j] = dist;
        floodArray[i][mazeSize - 1 - j] = dist;
        floodArray[mazeSize - 1 - i][mazeSize - 1 - j] = dist;
      }
  }
  // It is assumed we start at the bottom left corner : floodArray[mazeSize - 1][0] 
}



bool centerReached(){
  if (floodArray[currentPosition.x][currentPosition.y] == 0)
    return true;  
  else
    return false;  
}

// Function to update position based on current orientation
void updatePosition() {
  switch (currentOrientation) {
    case 0:  // Moving FORWARD (up)
      if (currentPosition.x > 0) { 
        currentPosition.x--;
      }
      break;
    case 1:  // Moving RIGHT (right)
      if (currentPosition.y < mazeSize - 1) { 
        currentPosition.y++;
      }
      break;
    case 2:  // Moving BACKWARD (down)
      if (currentPosition.x < mazeSize - 1) { 
        currentPosition.x++;
      }
      break;
    case 3:  // Moving LEFT (left)
      if (currentPosition.y > 0) { 
        currentPosition.y--;
      }
      break;
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////


void move() {
  // Reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;

  // Update the target angle
  mpu.update();
  //targetAngle = mpu.getAngleZ();

  while ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    mpu.update();
    float currentAngle = mpu.getAngleZ();
    Serial.print("Target "); 
    Serial.println(targetAngle);
    Serial.print("Current ");
    Serial.println(currentAngle);
    adjustMotorSpeeds(currentAngle, targetAngle);
   
    // Set motor directions to move forward
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);

    digitalWrite(motor2_in3, LOW );
    digitalWrite(motor2_in4, HIGH );

    // Apply speeds to motors
    //analogWrite(ENA, baseSpeed);
    //alogWrite(ENB, baseSpeed);
  }

  stopMotors();
  //Serial.println("Reached target distance.");

}



// PID parameters
float Kp = 7;  // Proportional gain 6 0 0 
float Ki = 0.5;  // Integral gain
float Kd = 0.5;  // Derivative gain

// PID variables
float integral = 0;
float lastError = 0;
float derivative = 0;

void adjustMotorSpeeds(float currentAngle, float targetAngle) {
  float error = targetAngle - currentAngle;
  integral += error;  // Accumulate the error over time
  derivative = error - lastError;  // Calculate the change in error

  int speedAdjustment = (int)(Kp * error + Ki * integral + Kd * derivative);  // PID formula
  //int speedAdjustment = 0;

  int leftSpeed = baseSpeed + speedAdjustment; // + is correct 
  int rightSpeed = baseSpeed - speedAdjustment;
  
  // Constrain speeds toc allowable range
  leftSpeed = constrain(leftSpeed, 70, 100);
  rightSpeed = constrain(rightSpeed, 70, 100);
   
  // Apply adjusted speeds
  // note : if right > left -> right
  // if left > right -> right 
  // follows the minimum
  analogWrite(ENA, leftSpeed); // left
  analogWrite(ENB, rightSpeed); // right 
    Serial.print("current angel "); 
    Serial.println(currentAngle);
    Serial.print("adjustment ");
    Serial.println(speedAdjustment);

    Serial.print("left "); 
    Serial.println(leftSpeed);
    Serial.print("Right ");
    Serial.println(rightSpeed);
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

void move_backward() {

  currentOrientation = (currentOrientation + 2) % 4;

   // Reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;

  // Update the target angle
  mpu.update();
  //targetAngle = mpu.getAngleZ();

  while ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    mpu.update();
    float currentAngle = mpu.getAngleZ();
    Serial.print("Target "); 
    Serial.println(targetAngle);
    Serial.print("Current ");
    Serial.println(currentAngle);
    adjustMotorSpeeds_back(currentAngle, targetAngle);
   
    // Set motor directions to move forward
    digitalWrite(motor1_in1, LOW);
    digitalWrite(motor1_in2, HIGH);

    digitalWrite(motor2_in3, HIGH );
    digitalWrite(motor2_in4, LOW );

    // Apply speeds to motors
    //analogWrite(ENA, 80);
    //analogWrite(ENB, 80);
  }

  stopMotors();
  //Serial.println("Reached target distance.");
  }

void adjustMotorSpeeds_back(float currentAngle, float targetAngle) {
  float error = targetAngle - currentAngle;
  integral += error;  // Accumulate the error over time
  derivative = error - lastError;  // Calculate the change in error

  int speedAdjustment = (int)(Kp * error + Ki * integral + Kd * derivative);  // PID formula
  //int speedAdjustment = 0;

  int leftSpeed = baseSpeed - speedAdjustment; // + is correct 
  int rightSpeed = baseSpeed + speedAdjustment;
  
  // Constrain speeds toc allowable range
  leftSpeed = constrain(leftSpeed, 70, 100);
  rightSpeed = constrain(rightSpeed, 70, 100);
   
  // Apply adjusted speeds
  // note : if right > left -> right
  // if left > right -> right 
  // follows the minimum
  analogWrite(ENA, leftSpeed); // left
  analogWrite(ENB, rightSpeed); // right 
    Serial.print("current angel "); 
    Serial.println(currentAngle);
    Serial.print("adjustment ");
    Serial.println(speedAdjustment);

    Serial.print("left "); 
    Serial.println(leftSpeed);
    Serial.print("Right ");
    Serial.println(rightSpeed);
  // Update lastError for the next cycle
  lastError = error;


}

void move_1() {
  // Reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;

  // Update the target angle
  mpu.update();
  //targetAngle = mpu.getAngleZ();

  while ((encoder1_count < 25) && (encoder2_count < 25)) {
    mpu.update();
    float currentAngle = mpu.getAngleZ();
    Serial.print("Target "); 
    Serial.println(targetAngle);
    Serial.print("Current ");
    Serial.println(currentAngle);
    adjustMotorSpeeds(currentAngle, targetAngle);
   
    // Set motor directions to move forward
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);

    digitalWrite(motor2_in3, LOW );
    digitalWrite(motor2_in4, HIGH );

    // Apply speeds to motors
    //analogWrite(ENA, baseSpeed);
    //alogWrite(ENB, baseSpeed);
  }

  stopMotors();
  //Serial.println("Reached target distance.");
}
