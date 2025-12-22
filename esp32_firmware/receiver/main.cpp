#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ============================================================================
// Pin Definitions
// ============================================================================
#define BATTERY_VOLTAGE_PIN 34  // Analog input for battery voltage sensor
#define UART_TX_PIN 17          // TX to STM32 RX (PA10)
#define UART_RX_PIN 16          // RX from STM32 TX (PA9) - optional
#define UART_BAUD 115200

// ============================================================================
// Battery Voltage Configuration
// ============================================================================
// Adjust voltage divider ratio based on your circuit
// Example: 10kΩ and 2.2kΩ divider for 12V → 3.3V
// Ratio = (10k + 2.2k) / 2.2k = 5.545
#define VOLTAGE_DIVIDER_RATIO 5.545
#define ADC_RESOLUTION 4095.0
#define ADC_REFERENCE_VOLTAGE 3.3

// ============================================================================
// Data Structures
// ============================================================================
typedef struct {
    uint16_t throttle;  // 1000-2000
    uint16_t steering;  // 1000-2000
    uint8_t buttons;    // Bitmask
} ControlData;

typedef struct {
    float batteryVoltage;
    uint8_t rssi;
} TelemetryData;

ControlData incomingControl;
TelemetryData outgoingTelemetry;

// Remote MAC address (set after you get it from the remote ESP32)
uint8_t remoteMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============================================================================
// UART Serial to STM32
// ============================================================================
HardwareSerial STM32Serial(1);  // Use UART1

// ============================================================================
// ESP-NOW Callbacks
// ============================================================================
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(ControlData)) {
        memcpy(&incomingControl, data, sizeof(ControlData));
        
        // Forward to STM32 via UART
        // Packet format: < Throttle_High Throttle_Low Steering_High Steering_Low >
        uint8_t packet[6];
        packet[0] = '<';
        packet[1] = (incomingControl.throttle >> 8) & 0xFF;  // High byte
        packet[2] = incomingControl.throttle & 0xFF;         // Low byte
        packet[3] = (incomingControl.steering >> 8) & 0xFF;
        packet[4] = incomingControl.steering & 0xFF;
        packet[5] = '>';
        
        STM32Serial.write(packet, 6);
        
        // Optional: Print for debugging
        Serial.printf("RX: T=%d S=%d Btn=0x%02X\n", 
                      incomingControl.throttle, 
                      incomingControl.steering, 
                      incomingControl.buttons);
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Track telemetry send status
}

// ============================================================================
// Battery Voltage Reading
// ============================================================================
float readBatteryVoltage() {
    int raw = analogRead(BATTERY_VOLTAGE_PIN);
    float voltage = (raw / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
    return voltage;
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ModuBOT Receiver (ESP32) ===");
    
    // Initialize UART to STM32
    STM32Serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    
    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    
    // Print MAC address for remote configuration
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        while (1);
    }
    
    // Register callbacks
    esp_now_register_recv_cb(onDataReceived);
    esp_now_register_send_cb(onDataSent);
    
    // Add remote as peer for telemetry TX
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, remoteMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Add Peer Failed");
    }
    
    Serial.println("Ready. Listening for control data...\n");
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
    // Read battery voltage
    outgoingTelemetry.batteryVoltage = readBatteryVoltage();
    outgoingTelemetry.rssi = 0;  // Optional: implement WiFi RSSI if needed
    
    // Send telemetry back to remote
    esp_now_send(remoteMAC, (uint8_t *)&outgoingTelemetry, sizeof(outgoingTelemetry));
    
    // Throttle telemetry updates to ~10 Hz
    delay(100);
}
