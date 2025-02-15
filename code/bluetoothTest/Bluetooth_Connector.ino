#define LED_PIN 13    // Built-in LED
#define BT_ENABLE_PIN 5  // Pin 5 to control HC-05 EN/KEY

void setup() {
    pinMode(LED_PIN, OUTPUT);    // Set built-in LED pin as output
    pinMode(BT_ENABLE_PIN, OUTPUT);  // Set Pin 5 as output for HC-05 enable

    Serial.begin(9600);          // Start hardware serial communication (TX/RX)
    Serial.println("Bluetooth control: Type '1' to turn LED ON, '0' to turn LED OFF");

    // Enable HC-05 by setting Pin 5 HIGH for AT Command Mode (if needed)
    digitalWrite(BT_ENABLE_PIN, HIGH);  // Enable HC-05 (AT mode for configuration)
    delay(1000);  // Wait for a moment before switching to data mode
    
    // Switch HC-05 to Data Communication Mode (important for normal operation)
    digitalWrite(BT_ENABLE_PIN, LOW);   // Set Pin 5 LOW to allow data communication
}

void loop() {
    // If data is available from HC-05 (through hardware serial)
    if (Serial.available()) {
        char command = Serial.read();  // Read the incoming data
        Serial.print("Received command: ");
        Serial.println(command);

        // If command is '1', turn LED ON and send current millis
        if (command == '1') { 
            digitalWrite(LED_PIN, HIGH);  // Turn LED on
            Serial.println("LED ON");
            Serial.print("Current millis: ");
            Serial.println(millis());  // Send the number of milliseconds since the Arduino started
        }
        // If command is '0', turn LED OFF
        else if (command == '0') { 
            digitalWrite(LED_PIN, LOW);   // Turn LED off
            Serial.println("LED OFF");
        }
    }
}
