/*
PIN CONNECTION GUIDE
===================

ESP32 Connections:
-----------------
GPIO16 -> Forward Button
GPIO17 -> Backward Button
GPIO18 -> Left Button
GPIO19 -> Right Button
GPIO22 -> Gas LED pin 1
GPIO23 -> Gas LED pin 2
GPIO21 -> Warning LED Yellow (with 220Ω resistor)
GND -> Button GND connections, LED GND connections

Buttons:
--------
Each button needs:
1. One terminal -> Corresponding ESP32 GPIO
2. Other terminal -> GND
3. 10kΩ pull-up resistor between GPIO and 3.3V
   (Not needed if using INPUT_PULLUP)

Forward Button:
- Terminal 1 -> GPIO16
- Terminal 2 -> GND
- Optional external pull-up: 10kΩ from GPIO16 to 3.3V

Backward Button:
- Terminal 1 -> GPIO17
- Terminal 2 -> GND
- Optional external pull-up: 10kΩ from GPIO17 to 3.3V

Left Button:
- Terminal 1 -> GPIO18
- Terminal 2 -> GND
- Optional external pull-up: 10kΩ from GPIO18 to 3.3V

Right Button:
- Terminal 1 -> GPIO19
- Terminal 2 -> GND
- Optional external pull-up: 10kΩ from GPIO19 to 3.3V

Dual-Color Gas LED:
--------------------
Red LED:
- Pin1 -> GPIO22 through 220Ω resistor

Green LED:
- Pin2 -> GPIO23 through 220Ω resistor

Warning LED (Yellow):
--------------------
- Anode -> GPIO21 through 220Ω resistor
- Cathode -> GND

Power Supply:
------------
1. ESP32 powered via USB or 5V regulated supply
2. If using battery power, add:
   - Power switch
   - Battery voltage monitoring (optional)
   - Polarity protection diode
   - 5V voltage regulator if using >5V battery

Notes:
1. All GND connections should be connected together
2. Add 0.1μF ceramic capacitor across power supply
3. Consider adding debouncing capacitors (0.1μF) across buttons
4. For battery operation, consider adding low battery detection
5. Optional: Add LED brightness control via PWM
*/

#include <esp_now.h>
#include <WiFi.h>

// Pin Definitions
#define BTN_FORWARD    16  // Forward button
#define BTN_BACKWARD   17  // Backward button
#define BTN_LEFT       18  // Left button
#define BTN_RIGHT      19  // Right button
#define GAS_LED_P1    22  // Gas LED Red pin
#define GAS_LED_P2    23  // Gas LED Green pin
#define WARNING_LED    21  // Yellow warning LED

// Constants
#define BLINK_INTERVAL 500  // LED blink interval in ms
#define GAS_THRESHOLD  2000 // Adjust based on your MQ135 calibration

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Global variables
unsigned long lastBlinkTime = 0;
bool ledState = false;
bool obstacleDetected = false;
bool forwardPressed = false;
//bot mac 88:13:bf:62:d3:30
uint8_t botAddress[] = {0x88, 0x13, 0xbf, 0x62, 0xd3, 0x30};

// Structure for receiving sensor data
struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
};

// Structure for sending commands
struct CommandData {
    Direction direction;
};
// Callback function for receiving sensor data
void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data_len == sizeof(SensorData)) {
        SensorData sensorData;
        memcpy(&sensorData, data, sizeof(sensorData));
        processSensorData(sensorData);
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(BTN_FORWARD, INPUT_PULLUP);
    pinMode(BTN_BACKWARD, INPUT_PULLUP);
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(GAS_LED_P1, OUTPUT);
    pinMode(GAS_LED_P2, OUTPUT);
    pinMode(WARNING_LED, OUTPUT);
    
    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    
    // Register callback
    esp_now_register_recv_cb(onDataReceived);
    
    // Add bot as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, botAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    // Check buttons and send commands
    checkButtons();
    
    // Handle warning LED
    handleWarningLED();
}


void checkButtons() {
    CommandData command;
    command.direction = Direction::Stop;  // Default to stop
    
    // Check each button and set appropriate direction
    if (!digitalRead(BTN_FORWARD)) {
        command.direction = Direction::Forward;
        Serial.println("Forward");
        forwardPressed = true;
    }
    else if (!digitalRead(BTN_BACKWARD)) {
        command.direction = Direction::Backward;
        Serial.println("Backward");
        forwardPressed = false;
    }
    else if (!digitalRead(BTN_LEFT)) {
        command.direction = Direction::Left;
        Serial.println("Turn left");
        forwardPressed = false;
    }
    else if (!digitalRead(BTN_RIGHT)) {
        command.direction = Direction::Right;
        Serial.println("Turn right");
        forwardPressed = false;
    }
    else {
        forwardPressed = false;
    }
    
    // Send command via ESP-NOW
    esp_now_send(botAddress, (uint8_t*)&command, sizeof(command));
}

void handleWarningLED() {
    if (obstacleDetected) {
        if (forwardPressed) {
            // Blink LED if trying to move forward with obstacle
            if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
                ledState = !ledState;
                digitalWrite(WARNING_LED, ledState);
                lastBlinkTime = millis();
            }
        } else {
            // Solid LED if obstacle detected but not moving
            digitalWrite(WARNING_LED, HIGH);
        }
    } else {
        digitalWrite(WARNING_LED, LOW);
    }
}

void processSensorData(const SensorData &data) {
    // Update obstacle status
    obstacleDetected = data.obstacle;
    
    // Update gas warning LED
    if (data.airQuality > GAS_THRESHOLD) {
        digitalWrite(GAS_LED_P2, LOW);
        digitalWrite(GAS_LED_P1, HIGH);
    } else {
        digitalWrite(GAS_LED_P2, HIGH);
        digitalWrite(GAS_LED_P1, LOW);
    }
    
    // Call custom function to handle sensor data
    handleSensorData(data);
}

// Custom function to handle sensor data (to be implemented based on needs)
void handleSensorData(const SensorData &data) {
    // Print data to Serial for debugging
    Serial.print("Temperature: "); Serial.print(data.temperature);
    Serial.print("°C, Humidity: "); Serial.print(data.humidity);
    Serial.print("%, Air Quality: "); Serial.print(data.airQuality);
    Serial.print(", Obstacle: "); Serial.println(data.obstacle ? "Yes" : "No");
    
    // Add your custom handling code here
    // This could include:
    // - Displaying on an LCD/OLED
    // - Sending to a server
    // - Logging to SD card
    // - Triggering additional actions
}