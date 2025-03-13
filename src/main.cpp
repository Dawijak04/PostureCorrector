/**
 * Posture Corrector
 * 
 * Monitors posture using an MPU6050 accelerometer and provides
 * feedback through a buzzer when slouching is detected.
 * 
 * Hardware:
 * - ESP32
 * - MPU6050 accelerometer
 * - Buzzer on PIN 15
 */

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFi.h>

// WiFi Configuration
const char* const WIFI_SSID = "Dawids S22 Ultra";
const char* const WIFI_PASSWORD = "Bond2004";
const char* const REMOTE_HOST = "google.com";
constexpr int REMOTE_PORT = 80;

// Hardware Configuration
constexpr int BUZZER_PIN = 15;
constexpr int BUZZER_CHANNEL = 0;
constexpr int BUZZER_FREQUENCY = 2000;  // Hz
constexpr int BUZZER_RESOLUTION = 8;    // bits
constexpr int BUZZER_DUTY_CYCLE = 127;  // ~50% of 8-bit (0-255)
constexpr int BUZZER_DURATION = 100;    // ms
constexpr int BUZZER_PAUSE = 50;        // ms

// System Configuration
constexpr int WIFI_CONNECTION_TIMEOUT = 20;  // seconds
constexpr int SERIAL_BAUD_RATE = 115200;
constexpr int SAMPLING_INTERVAL = 50;        // ms

// Posture Detection Configuration
constexpr int SLOUCH_THRESHOLD = 10;         // Number of consecutive readings needed
constexpr int CALIBRATION_SAMPLES_REQUIRED = 5;
constexpr int SLOUCHING_THRESHOLD = -14900;   // Acceleration threshold for slouching

// Enum for calibration status
enum class CalibrationStatus {
  IN_PROGRESS,
  COMPLETE
};

// Global state
struct {
  MPU6050 mpu;
  struct {
    int16_t x, y, z;
  } acceleration;
  struct {
    int16_t x, y, z;
  } gyro;
  int16_t slouchCount = 0;
  bool isCalibrated = false;
  int calibrationCount = 0;
} state;

/**
 * Attempts to connect to the configured WiFi network.
 * 
 * @return true if the connection was successful, false otherwise
 */
bool setupWiFi() {
  Serial.println("\nStarting WiFi setup...");
  WiFi.disconnect();
  
  Serial.printf("Connecting to: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECTION_TIMEOUT) {
    delay(1000);
    Serial.print(".");
    attempts++;
    
    if (attempts % 5 == 0) {
      Serial.printf("\nStatus: %d\n", WiFi.status());
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi Connection Failed!");
    Serial.print("Error: ");
    
    switch (WiFi.status()) {
      case WL_IDLE_STATUS:      Serial.println("Idle"); break;
      case WL_NO_SSID_AVAIL:    Serial.println("SSID not found"); break;
      case WL_CONNECT_FAILED:   Serial.println("Wrong password"); break;
      case WL_DISCONNECTED:     Serial.println("Cannot reach network"); break;
      default:                  Serial.println(WiFi.status());
    }
    
    return false;
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

/**
 * Tests if the device can reach the internet.
 * 
 * @return true if internet connection is available, false otherwise
 */
bool testInternetConnection() {
  WiFiClient client;
  Serial.printf("Testing connection to %s\n", REMOTE_HOST);
  
  if (client.connect(REMOTE_HOST, REMOTE_PORT)) {
    Serial.println("Connection test successful");
    client.stop();
    return true;
  }
  
  Serial.println("Connection test failed");
  return false;
}

/**
 * Initializes the MPU6050 sensor.
 * 
 * @return true if initialization was successful, false otherwise
 */
bool initializeMPU() {
  Wire.begin();
  state.mpu.initialize();
  
  if (state.mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
    return true;
  }
  
  Serial.println("MPU6050 connection failed!");
  return false;
}

/**
 * Activates the buzzer to alert the user.
 */
void activateBuzzer() {
  ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY_CYCLE);
  delay(BUZZER_DURATION);
  ledcWrite(BUZZER_CHANNEL, 0);  // Ensure buzzer is off
  delay(BUZZER_PAUSE);
}

/**
 * Manages the posture calibration process.
 * Calibration ensures the device recognizes the user's proper posture.
 * 
 * @return CalibrationStatus::COMPLETE if calibration is complete, CalibrationSt  atus::IN_PROGRESS otherwise
 */
CalibrationStatus updateCalibration() {
  const bool isGoodPosture = state.acceleration.x < SLOUCHING_THRESHOLD;
  
  if (isGoodPosture) {
    state.calibrationCount++;
    Serial.printf(" Calibrating... %d/%d", state.calibrationCount, CALIBRATION_SAMPLES_REQUIRED);
    
    if (state.calibrationCount >= CALIBRATION_SAMPLES_REQUIRED) {
      Serial.println(" CALIBRATION COMPLETE");
      return CalibrationStatus::COMPLETE;
    }
  } else {
    state.calibrationCount = 0;
    Serial.println(" Please sit up straight for calibration");
  }
  
  return CalibrationStatus::IN_PROGRESS;
}

/**
 * Monitors the user's posture and provides feedback when slouching is detected.
 */
void monitorPosture() {
  const bool isSlouchingDetected = state.acceleration.x >= SLOUCHING_THRESHOLD;
  
  if (isSlouchingDetected) {
    state.slouchCount++;
    if (state.slouchCount >= SLOUCH_THRESHOLD) {
      Serial.println(" SLOUCHING DETECTED");
      activateBuzzer();
    }
  } else {
    state.slouchCount = 0;
  }
}

/**
 * Reads the current accelerometer values.
 */
void updateSensorReadings() {
  state.mpu.getAcceleration(
    &state.acceleration.x, 
    &state.acceleration.y, 
    &state.acceleration.z
  );
  
  Serial.printf("AccelX: %d", state.acceleration.x);
}

/**
 * Setup function, runs once when the device starts.
 */
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Initialize buzzer with PWM
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQUENCY, BUZZER_RESOLUTION);
  
  const bool isSystemReady = setupWiFi() && 
                           testInternetConnection() && 
                           initializeMPU();
  
  if (!isSystemReady) {
    Serial.println("Initialization failed. System halted.");
    while (true) {
      delay(1000);  // Halt execution
    }
  }
}

/**
 * Main program loop, runs repeatedly after setup.
 */
void loop() {
  // Ensure buzzer is off at the start of each loop cycle
  ledcWrite(BUZZER_CHANNEL, 0);
  
  updateSensorReadings();
  
  if (!state.isCalibrated) {
    const auto status = updateCalibration();
    state.isCalibrated = (status == CalibrationStatus::COMPLETE);
  } else {
    monitorPosture();
  }
  
  Serial.println();
  delay(SAMPLING_INTERVAL);
}