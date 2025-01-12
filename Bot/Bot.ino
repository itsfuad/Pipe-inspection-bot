#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

// Pin Definitions
#define MOTOR_LEFT_FWD    18  // L293D pin 2
#define MOTOR_LEFT_BWD    19  // L293D pin 7
#define MOTOR_RIGHT_FWD   22  // L293D pin 10
#define MOTOR_RIGHT_BWD   23  // L293D pin 15
#define DHT_PIN           21  // DHT11 data pin
#define MQ135_PIN         34  // MQ135 analog pin
#define TRIG_PIN          13  // HC-SR04 trigger
#define ECHO_PIN          14  // HC-SR04 echo
#define WARNING_LED       27  // Yellow LED for obstacle warning

// Constants
#define OBSTACLE_THRESHOLD 20   // Distance in cm
#define DHT_TYPE DHT11
#define BLINK_INTERVAL 100      // LED blink interval in ms
#define ULTRASONIC_TIMEOUT 30000  // Microseconds

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Global variables
DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastBlinkTime = 0;
bool ledState = false;
bool obstacleDetected = false;
bool forwardCommandReceived = false;

// MAC address of the last device that sent a command
uint8_t remoteAddress[6];
bool hasRemoteAddress = false;

// Structure for sending sensor data
struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
};

// Structure for receiving commands
struct CommandData {
    Direction direction;
};

// Command callback function
void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    memcpy(remoteAddress, esp_now_info->src_addr, 6);
    hasRemoteAddress = true;

    if (data_len == sizeof(CommandData)) {
        CommandData command;
        memcpy(&command, data, sizeof(command));
        processCommand(command.direction);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_BWD, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_BWD, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(WARNING_LED, OUTPUT);
    pinMode(MQ135_PIN, INPUT);

    // Initialize DHT sensor
    dht.begin();

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onDataReceived);

    // Print header for Serial Plotter
    Serial.println("Temperature\tHumidity\tAirQuality\tDistance");
}

void loop() {
    // Read sensors
    readSensors();

    // Check for obstacles
    checkObstacles();

    // Handle warning LED
    handleWarningLED();
}

void readSensors() {
    static unsigned long lastSendTime = 0;
    const unsigned long SEND_INTERVAL = 2000; // Send every 2 seconds

    if (millis() - lastSendTime >= SEND_INTERVAL) {
        SensorData sensorData;

        // Read DHT11
        sensorData.temperature = dht.readTemperature();
        sensorData.humidity = dht.readHumidity();

        // Read MQ135
        sensorData.airQuality = analogRead(MQ135_PIN);

        // Set obstacle status
        sensorData.obstacle = obstacleDetected;

        // Send data via ESP-NOW
        if (hasRemoteAddress) {
            esp_now_send(remoteAddress, (uint8_t *)&sensorData, sizeof(sensorData));
        }

        // Print sensor data for Serial Plotter
        Serial.printf("%.1f\t%.1f\t%d\t", sensorData.temperature, sensorData.humidity, sensorData.airQuality);
        Serial.print(obstacleDetected ? "1\n" : "0\n");  // Use 1 for obstacle, 0 otherwise

        lastSendTime = millis();
    }
}

void checkObstacles() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
    float distance = duration * 0.034 / 2;

    // Update obstacle status
    obstacleDetected = (distance > 0 && distance < OBSTACLE_THRESHOLD);

    // Print distance to Serial Plotter
    Serial.printf("%.2f\t", distance > 0 ? distance : 0);
}

void handleWarningLED() {
    if (obstacleDetected) {
        if (forwardCommandReceived) {
            // Blink LED
            if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
                ledState = !ledState;
                digitalWrite(WARNING_LED, ledState);
                lastBlinkTime = millis();
            }
        } else {
            // Solid LED
            digitalWrite(WARNING_LED, HIGH);
        }
    } else {
        digitalWrite(WARNING_LED, LOW);
    }
}

void processCommand(Direction direction) {
    forwardCommandReceived = (direction == Direction::Forward);

    if (obstacleDetected && direction == Direction::Forward) {
        // Stop if obstacle detected
        stopMotors();
        return;
    }

    switch (direction) {
        case Direction::Stop:
            stopMotors();
            break;
        case Direction::Forward:
            moveForward();
            break;
        case Direction::Backward:
            moveBackward();
            break;
        case Direction::Left:
            turnLeft();
            break;
        case Direction::Right:
            turnRight();
            break;
    }

    //Serial.printf("Command: %d\n", static_cast<int>(direction));
}

void stopMotors() {
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveForward() {
    digitalWrite(MOTOR_LEFT_FWD, HIGH);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, HIGH);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveBackward() {
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_BWD, HIGH);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void turnLeft() {
    digitalWrite(MOTOR_LEFT_FWD, LOW);
    digitalWrite(MOTOR_LEFT_BWD, HIGH);
    digitalWrite(MOTOR_RIGHT_FWD, HIGH);
    digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnRight() {
    digitalWrite(MOTOR_LEFT_FWD, HIGH);
    digitalWrite(MOTOR_LEFT_BWD, LOW);
    digitalWrite(MOTOR_RIGHT_FWD, LOW);
    digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}
