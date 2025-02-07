#define motor1_in1 12
#define motor1_in2 14
#define motor2_in3 27
#define motor2_in4 26

#define ENA 13
#define ENB 25

#define encoder1_a 4
#define encoder2_a 19

volatile int encoder1_count = 0;
volatile int encoder2_count = 0;

void IRAM_ATTR encoder1_isr() {
  encoder1_count++;
}

void IRAM_ATTR encoder2_isr() {
  encoder2_count++;
}

void setup() {
  Serial.begin(115200);

  // Set motor pins as outputs
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set encoder pins as inputs
  pinMode(encoder1_a, INPUT_PULLUP);
  pinMode(encoder2_a, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoder1_a), encoder1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_a), encoder2_isr, RISING);

  Serial.println("Starting Motor Test...");
}

void loop() {
  encoder1_count = 0;
  encoder2_count = 0;

  // Run both motors at the same speed
  int speed = 180;  // Adjust if needed
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  delay(2000); // Run motors for 2 seconds

  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Print encoder counts
  Serial.print("Encoder1: ");
  Serial.print(encoder1_count);
  Serial.print(" | Encoder2: ");
  Serial.print(encoder2_count);

  int difference = encoder1_count - encoder2_count;
  Serial.print(" | Difference: ");
  Serial.println(difference);


  delay(1000);
}
