#include <esp_now.h>
#include <WiFi.h>

// Motor A pins (Left track)
#define MOTOR_A_PIN1 5
#define MOTOR_A_PIN2 18
#define MOTOR_A_ENABLE 19

// Motor B pins (Right track)
#define MOTOR_B_PIN1 23
#define MOTOR_B_PIN2 22
#define MOTOR_B_ENABLE 21

// MQ135 gas sensor pin
#define MQ135_PIN 34 // ADC pin for gas sensor

// Enum for button signals
enum class ButtonSignal : uint8_t {
  UP,
  DOWN,
  LEFT,
  RIGHT,
  STOP
};

// Struct to hold sent data (button signal + gas sensor reading)
typedef struct {
  ButtonSignal signal; // Signal from sender
  float gasLevel;      // Gas level reading from MQ135
} ButtonSignalData;

ButtonSignalData sendData;

// Function to control the motors
void moveMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
  digitalWrite(pin1, forward ? HIGH : LOW);
  digitalWrite(pin2, forward ? LOW : HIGH);
  analogWrite(enablePin, speed);
}

// Function to stop a motor
void stopMotor(int enablePin) {
  analogWrite(enablePin, 0);
}

void moveTank(ButtonSignal signal) {
  int fullSpeed = 255; // Full motor power

  switch (signal) {
    case ButtonSignal::UP: // Move forward
      Serial.println("Command: Move Forward");
      moveMotor(MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_ENABLE, true, fullSpeed);
      moveMotor(MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_ENABLE, true, fullSpeed);
      break;

    case ButtonSignal::DOWN: // Move backward
      Serial.println("Command: Move Backward");
      moveMotor(MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_ENABLE, false, fullSpeed);
      moveMotor(MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_ENABLE, false, fullSpeed);
      break;

    case ButtonSignal::LEFT: // Spin left
      Serial.println("Command: Spin Left");
      moveMotor(MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_ENABLE, false, fullSpeed);
      moveMotor(MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_ENABLE, true, fullSpeed);
      break;

    case ButtonSignal::RIGHT: // Spin right
      Serial.println("Command: Spin Right");
      moveMotor(MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_ENABLE, true, fullSpeed);
      moveMotor(MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_ENABLE, false, fullSpeed);
      break;

    case ButtonSignal::STOP: // Stop the tank
      Serial.println("Command: Stop");
      stopMotor(MOTOR_A_ENABLE);
      stopMotor(MOTOR_B_ENABLE);
      break;

    default:
      Serial.println("Command: Unknown");
      break;
  }
}


// Callback for received data
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&sendData, incomingData, sizeof(sendData));
  Serial.print("Received signal: ");

  // Handle the received signal
  switch (sendData.signal) {
    case ButtonSignal::UP: Serial.println("UP"); break;
    case ButtonSignal::DOWN: Serial.println("DOWN"); break;
    case ButtonSignal::LEFT: Serial.println("LEFT"); break;
    case ButtonSignal::RIGHT: Serial.println("RIGHT"); break;
    case ButtonSignal::STOP: Serial.println("STOP"); break;
    default: Serial.println("UNKNOWN"); break;
  }

  // Move the tank based on the received signal
  moveTank(sendData.signal);
}

// Read gas data from MQ135
float readGasLevel() {
  int sensorValue = analogRead(MQ135_PIN);
   return sensorValue * (3.3 / 4095.0) * 1000.0; // Adjust conversion factor as needed
}

// Callback for sending data
void sendDataCallback() {
  sendData.gasLevel = readGasLevel();
  Serial.printf("Gas level from car: %f\n", sendData.gasLevel);
  esp_now_send(NULL, (uint8_t *)&sendData, sizeof(sendData)); // Broadcast data
}

// Setup function
void setup() {
  Serial.begin(115200);

  // Motor pins setup
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_A_ENABLE, OUTPUT);

  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);
  pinMode(MOTOR_B_ENABLE, OUTPUT);

  // Stop motors initially
  stopMotor(MOTOR_A_ENABLE);
  stopMotor(MOTOR_B_ENABLE);

  // Initialize gas sensor
  pinMode(MQ135_PIN, INPUT);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onDataReceive);

  Serial.println("ESP-NOW Receiver Initialized");
}

// Main loop
void loop() {
  sendDataCallback();
  delay(100); // Send data every 1 second
}
