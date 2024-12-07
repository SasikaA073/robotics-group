// This is the finalizes ESP32 Code,(Upload it and you’ll get an ip adress, note it down)

#include <WiFi.h>
#include <ESP32Servo.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "1Router2Girls";
const char* password = "freewifi";

// Create a web server on port 80
WebServer server(80);

// Create servo objects for five joints and the gripper
Servo servo1, servo2, servo3, servo4, servo5, gripperServo;

double now_time = 0;

// Define GPIO pins for the servos
const int servoPin1 = 18;
const int servoPin2 = 19;
const int servoPin3 = 5;
const int servoPin4 = 23;
const int servoPin5 = 15;
const int gripperPin = 4;

// Current positions of servos in microseconds
int currentPositions[6] = {1500, 1000, 2000, 1600, 2500, 2200};

// Smooth movement parameters
const int duration = 2000;     // Duration of the movement in milliseconds
const int updateInterval = 1; // Interval to update servo positions in milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize WiFi
  connectToWiFi();

  // Attach servos to pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  gripperServo.attach(gripperPin);

  // Register server handlers
  server.on("/set_positions", handleSetPositions);

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Move servos to starting positions
  moveToPositions(currentPositions);
}

void loop() {
  // Handle incoming client requests
  server.handleClient();
}

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println("ESP32 IP Address: " + WiFi.localIP().toString());
}

// Function to handle the GET request
void handleSetPositions() {
  if (server.hasArg("pos")) {
    String positionsArg = server.arg("pos");
    int newPositions[6];

    // Parse positions from the GET parameter
    if (parsePositions(positionsArg, newPositions)) {
      // Smoothly move to the new positions
      smoothMove(
        servo1, servo2, servo3, servo4, servo5, gripperServo,
        currentPositions, newPositions,
        duration);

      // Update current positions
      for (int i = 0; i < 6; i++) {
        currentPositions[i] = newPositions[i];
      }
      server.send(200, "text/plain", "Positions updated");
    } else {
      server.send(400, "text/plain", "Invalid position data");
    }
  } else {
    server.send(400, "text/plain", "Missing 'pos' parameter");
  }
}

// Function to parse positions from the input string
bool parsePositions(String input, int positions[]) {
  int index = 0;
  int startIndex = 0;
  int commaIndex;

  while ((commaIndex = input.indexOf(',', startIndex)) != -1 && index < 6) {
    positions[index++] = input.substring(startIndex, commaIndex).toInt();
    startIndex = commaIndex + 1;
  }

  if (index < 5 || startIndex >= input.length()) {
    return false; // Invalid input format
  }

  positions[index++] = input.substring(startIndex).toInt(); // Add the last value
  return true;
}

// Function to move servos to specified positions
void moveToPositions(int positions[]) {
  servo1.writeMicroseconds(positions[0]);
  servo2.writeMicroseconds(positions[1]);
  servo3.writeMicroseconds(positions[2]);
  servo4.writeMicroseconds(positions[3]);
  servo5.writeMicroseconds(positions[4]);
  gripperServo.writeMicroseconds(positions[5]);
}

// Smooth movement function for 5 servos and gripper
void smoothMove(
  Servo &s1, Servo &s2, Servo &s3, Servo &s4, Servo &s5, Servo &gripper,
  int start[], int end[], int duration) {
  int steps = duration / updateInterval;  // Number of steps for interpolation
  float increments[6];                    // Array to hold step increments for each servo including gripper

  // Calculate increments for each servo
  for (int i = 0; i < 6; i++) {
    increments[i] = (float)(end[i] - start[i]) / steps;
  }

  // Perform smooth interpolation
  for (int i = 0; i <= steps; i++) {
    // Compute intermediate positions for each servo
    int currentPositions[6] = {
      start[0] + increments[0] * i,
      start[1] + increments[1] * i,
      start[2] + increments[2] * i,
      start[3] + increments[3] * i,
      start[4] + increments[4] * i,
      start[5] + increments[5] * i  // Gripper position
    };

    now_time = millis();

    while (millis() - now_time < updateInterval){

    // Set servos to computed positions
    s1.writeMicroseconds(currentPositions[0]);
    s2.writeMicroseconds(currentPositions[1]);
    s3.writeMicroseconds(currentPositions[2]);
    s4.writeMicroseconds(currentPositions[3]);
    s5.writeMicroseconds(currentPositions[4]);
    gripper.writeMicroseconds(currentPositions[5]);

    }
  }
}