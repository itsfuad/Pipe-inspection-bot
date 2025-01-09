#include <esp_now.h>
#include <WiFi.h>

// Define button pins
#define UP_BUTTON 12
#define DOWN_BUTTON 13
#define LEFT_BUTTON 14
#define RIGHT_BUTTON 27

// Define LED pins
#define LED_PIN1 25  // Dual-LED positive terminal
#define LED_PIN2 26  // Dual-LED negative terminal

// Define the enum class for button signals
enum class ButtonSignal : uint8_t {
  UP,
  DOWN,
  LEFT,
  RIGHT,
  STOP
};


// Define the struct for the data to be sent
typedef struct {
  ButtonSignal signal; // Use the enum class
} ButtonSignalData;

// Define the struct to receive the gas data
typedef struct {
  float gasValue;  // Gas sensor value
} GasSensorData;

uint8_t peerAddress[] = {0x88, 0x13, 0xBF, 0x62, 0xD3, 0x30}; // Replace with receiver's MAC address

ButtonSignalData signalData;

GasSensorData receivedGasData; 

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Data sent status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Function to control dual-LED polarity
void setDualLedColor(bool isColor1) {
  if (isColor1) {
    digitalWrite(LED_PIN1, HIGH);
    digitalWrite(LED_PIN2, LOW);
  } else {
    digitalWrite(LED_PIN1, LOW);
    digitalWrite(LED_PIN2, HIGH);
  }
}

// Callback for received data
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedGasData, incomingData, sizeof(receivedGasData));
  Serial.print("Received Gas Value: ");
  Serial.println(receivedGasData.gasValue);  // Print the received gas data
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set up button pins as input with pull-up
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  // Set up LED pins as output
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

  // Turn off dual-LED initially
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(onDataSent);

  // Register receive callback
  esp_now_register_recv_cb(onDataReceive);

  // Add the receiver's peer information
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW initialized");
}

void sendSignal(ButtonSignal signal) {
  // Turn on the dual-LED with color indicating "button pressed"
  setDualLedColor(true);

  // Set the signal data
  signalData.signal = signal;

  // Send the signal
  esp_now_send(peerAddress, (uint8_t *)&signalData, sizeof(signalData));

  // Change the dual-LED to indicate "data sent"
  setDualLedColor(false);

  // Turn off the LED after signaling
  delay(200); // Debounce delay
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
}

void loop() {
  // Track button states
  static bool upPressed = false, downPressed = false, leftPressed = false, rightPressed = false;

  // Check each button and send the appropriate signal
  if (digitalRead(UP_BUTTON) == LOW) {
    if (!upPressed) {
      sendSignal(ButtonSignal::UP);
      upPressed = true;
    }
  } else if (upPressed) {
    sendSignal(ButtonSignal::STOP);
    upPressed = false;
  }

  if (digitalRead(DOWN_BUTTON) == LOW) {
    if (!downPressed) {
      sendSignal(ButtonSignal::DOWN);
      downPressed = true;
    }
  } else if (downPressed) {
    sendSignal(ButtonSignal::STOP);
    downPressed = false;
  }

  if (digitalRead(LEFT_BUTTON) == LOW) {
    if (!leftPressed) {
      sendSignal(ButtonSignal::LEFT);
      leftPressed = true;
    }
  } else if (leftPressed) {
    sendSignal(ButtonSignal::STOP);
    leftPressed = false;
  }

  if (digitalRead(RIGHT_BUTTON) == LOW) {
    if (!rightPressed) {
      sendSignal(ButtonSignal::RIGHT);
      rightPressed = true;
    }
  } else if (rightPressed) {
    sendSignal(ButtonSignal::STOP);
    rightPressed = false;
  }
}

