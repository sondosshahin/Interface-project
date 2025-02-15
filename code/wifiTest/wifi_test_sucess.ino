#include <WiFi.h>
#include <WiFiUdp.h>


const char *ssid = "BANANA";     // Your Wi-Fi SSID
const char *password = "ABUHAMZA";  // Your Wi-Fi password

// Define UDP object
WiFiUDP udp;

const int listenPort = 12345; // Port for receiving messages

// Variable to store sender's address
IPAddress senderIP;
uint16_t senderPort;

void executeCommand(String receivedMessage);
void setUpWifi();
void receivePacket();  // Function prototype added

void setup() {
  Serial.begin(115200);
  setUpWifi(); 
}

void loop() {
  receivePacket();  
  delay(100);
}

void setUpWifi() {
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // Print the connected IP address
  Serial.println();
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start listening for UDP messages
  udp.begin(listenPort);
}

void receivePacket() {  
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Create a buffer to hold the incoming message
    char incomingPacket[packetSize + 1];  // +1 for the null terminator
    udp.read(incomingPacket, packetSize);

    // Null-terminate the string to prevent garbage data
    incomingPacket[packetSize] = '\0';

    // Store sender's IP and port
    senderIP = udp.remoteIP();
    senderPort = udp.remotePort();

    // Create a String object and trim unwanted characters
    String receivedMessage = incomingPacket;
    receivedMessage.trim();

    // Print received message
    Serial.print("Received message: ");
    Serial.println(receivedMessage);

    executeCommand(receivedMessage); 

    // Send acknowledgment (ack:<message>)
    String ackMessage = "ack:" + receivedMessage;
    udp.beginPacket(senderIP, senderPort);
    udp.write((uint8_t*)ackMessage.c_str(), ackMessage.length());
    udp.endPacket();

    Serial.print("Sent acknowledgment: ");
    Serial.println(ackMessage);
  }
}

void executeCommand(String receivedMessage) {
  // Implement your logic here for executing commands
}
