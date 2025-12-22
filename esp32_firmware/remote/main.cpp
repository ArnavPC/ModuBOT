#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

// ============================================================================
// LILYGO T-Display Pin Definitions
// ============================================================================
#define JOYSTICK_THROTTLE_PIN 32  // Left stick Y-axis (Throttle)
#define JOYSTICK_STEERING_PIN 36  // Right stick X-axis (Steering, VP)
#define BUTTON_1_PIN 17
#define BUTTON_2_PIN 21
#define BUTTON_3_PIN 22
#define BUTTON_4_PIN 27

// TFT_eSPI is configured via platformio.ini build flags for T-Display

// ============================================================================
// ESP-NOW Configuration
// ============================================================================
// Replace with your receiver ESP32 MAC address
// Find it by running WiFi.macAddress() on the receiver
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============================================================================
// Data Structures
// ============================================================================
typedef struct {
    uint16_t throttle;  // 1000-2000
    uint16_t steering;  // 1000-2000
    uint8_t buttons;    // Bitmask: bit 0-3 for buttons 1-4
} ControlData;

typedef struct {
    float batteryVoltage;  // Voltage in volts
    uint8_t rssi;          // Signal strength (optional)
} TelemetryData;

ControlData outgoingData;
TelemetryData incomingTelemetry;

// ============================================================================
// Display
// ============================================================================
TFT_eSPI tft = TFT_eSPI();

// ============================================================================
// ESP-NOW Callbacks
// ============================================================================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Track send success/failure
}

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(TelemetryData)) {
        memcpy(&incomingTelemetry, data, sizeof(TelemetryData));
    }
}

// ============================================================================
// Utility Functions
// ============================================================================
uint16_t mapJoystick(int raw) {
    // Map ADC range (0-4095) to RC PWM range (1000-2000)
    // Assumes center is ~2048; adjust if needed
    return constrain(map(raw, 0, 4095, 1000, 2000), 1000, 2000);
}

uint8_t readButtons() {
    uint8_t mask = 0;
    if (digitalRead(BUTTON_1_PIN) == LOW) mask |= (1 << 0);
    if (digitalRead(BUTTON_2_PIN) == LOW) mask |= (1 << 1);
    if (digitalRead(BUTTON_3_PIN) == LOW) mask |= (1 << 2);
    if (digitalRead(BUTTON_4_PIN) == LOW) mask |= (1 << 3);
    return mask;
}

void updateDisplay() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    
    tft.setCursor(10, 10);
    tft.println("ModuBOT Remote");
    
    tft.setCursor(10, 40);
    tft.print("Throttle: ");
    tft.println(outgoingData.throttle);
    
    tft.setCursor(10, 65);
    tft.print("Steering: ");
    tft.println(outgoingData.steering);
    
    tft.setCursor(10, 90);
    tft.print("Battery: ");
    tft.print(incomingTelemetry.batteryVoltage, 2);
    tft.println(" V");
    
    tft.setCursor(10, 115);
    tft.print("Buttons: 0x");
    tft.println(outgoingData.buttons, HEX);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    
    // Initialize display
    tft.init();
    tft.setRotation(1);  // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 10);
    tft.println("Initializing...");
    
    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        tft.setCursor(10, 40);
        tft.setTextColor(TFT_RED);
        tft.println("ESP-NOW Init Failed");
        while (1);
    }
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    
    // Add peer (receiver)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;  // Use current channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        tft.setCursor(10, 70);
        tft.setTextColor(TFT_RED);
        tft.println("Add Peer Failed");
        while (1);
    }
    
    tft.setCursor(10, 40);
    tft.setTextColor(TFT_GREEN);
    tft.println("Ready!");
    delay(1000);
    
    incomingTelemetry.batteryVoltage = 0.0;
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
    // Read joysticks
    int throttleRaw = analogRead(JOYSTICK_THROTTLE_PIN);
    int steeringRaw = analogRead(JOYSTICK_STEERING_PIN);
    
    outgoingData.throttle = mapJoystick(throttleRaw);
    outgoingData.steering = mapJoystick(steeringRaw);
    outgoingData.buttons = readButtons();
    
    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&outgoingData, sizeof(outgoingData));
    
    // Update display
    updateDisplay();
    
    // Throttle loop to ~50 Hz
    delay(20);
}
