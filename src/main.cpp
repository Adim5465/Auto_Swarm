#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP085.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>
#include <TinyGPS++.h>  // Include TinyGPS++ library
#include <HardwareSerial.h>  // For GPS UART communication
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Motor GPIO Pins
#define MOTOR_FL 12  // Front Left Motor
#define MOTOR_FR 13  // Front Right Motor
#define MOTOR_BL 11  // Back Left Motor
#define MOTOR_BR 14  // Back Right Motor

// ESC Configuration
#define ESC_MIN 1000  // Minimum throttle value for ESCs
#define ESC_MAX 2000  // Maximum throttle value for ESCs
#define BASE_THROTTLE 1380  // Base throttle for hover

// PID Constants for Roll, Pitch, Yaw, and Altitude
float Kp_roll = 0.1, Ki_roll = 0.012, Kd_roll = 10;
float Kp_pitch = 0.1, Ki_pitch = 0.012, Kd_pitch = 6.8;
float Kp_yaw = 0.4,Ki_yaw = 0, Kd_yaw = 0;
float Kp_altitude = 1, Ki_altitude = 0, Kd_altitude = 0;
float Kp_position = 0, Ki_position = 0, Kd_position = 0;  // PID for position hold

// Target Altitude (12 inches in meters)
const float TARGET_ALTITUDE = 0.3048;

// Safety Thresholds
const float SAFETY_ANGLE_THRESHOLD = 30.0;  // Stop motors if roll or pitch exceeds this angle
const float LOW_BATTERY_VOLTAGE = 10.0;     // Stop motors if battery voltage drops below this value

// Sensor Objects
MPU6050 mpu;  // MPU6050 for roll, pitch, and yaw
Adafruit_BMP085 bmp;  // BMP180 for altitude/pressure (not used for low-altitude control)
QMC5883LCompass compass;  // QMC5883L for heading/yaw
Servo escFL, escFR, escBL, escBR;
VL53L0X vl53;  // VL53L0X distance sensor for precise altitude control
TinyGPSPlus gps;  // GPS object
HardwareSerial gpsSerial(1);  // Use UART1 for GPS (TX = GPIO16, RX = GPIO17)

// PID Variables for Roll, Pitch, Yaw, Altitude, and Position
float roll_error_sum = 0, last_roll_error = 0;
float pitch_error_sum = 0, last_pitch_error = 0;
float yaw_error_sum = 0, last_yaw_error = 0;
float altitude_error_sum = 0, last_altitude_error = 0;
float position_error_sum = 0, last_position_error = 0;

// Drone State
enum DroneState { IDLE, TAKEOFF, HOVER, LANDING, STOP };
DroneState droneState = IDLE;

// Timers for State Management
unsigned long takeoffStartTime = 0;
unsigned long hoverStartTime = 0;
const unsigned long TAKEOFF_DURATION = 5000;  // 5 seconds for takeoff
const unsigned long HOVER_DURATION = 20000;   // 20 seconds for hover

// Shared Data (Protected by Mutex)
float roll = 0, pitch = 0, yaw = 0, altitude = 0;
float ground_altitude = 0;  // Ground altitude at takeoff
float battery_voltage = 12.6;  // Initial battery voltage (example value)
double target_latitude = 0, target_longitude = 0;  // Target position for position hold
SemaphoreHandle_t xMutex = NULL;

// Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t stateTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;

// Function to set ESC throttle values
void setThrottle(int fl, int fr, int bl, int br) {
  escFL.writeMicroseconds(fl);
  escFR.writeMicroseconds(fr);
  escBL.writeMicroseconds(bl);
  escBR.writeMicroseconds(br);
}

// Function to compute PID with anti-windup
float computePID(float error, float &error_sum, float &last_error, float dt, float Kp, float Ki, float Kd) {
  error_sum += error * dt;
  float d_error = (error - last_error) / dt;
  last_error = error;

  // Anti-windup: Clamp the integral term
  error_sum = constrain(error_sum, -100.0 / Ki, 100.0 / Ki);

  return (Kp * error) + (Ki * error_sum) + (Kd * d_error);
}

// Function to read sensor data
void readSensorData() {
  // Read MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float current_roll = atan2(ay, az) * 180 / PI;  // Roll from MPU6050
  float current_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;  // Pitch from MPU6050

  // Read magnetometer data (heading/yaw)
  compass.read();
  float current_yaw = compass.getAzimuth();  // Yaw from QMC5883L

  // Read VL53L0X data (altitude)
  float current_altitude = vl53.readRangeContinuousMillimeters() / 1000.0;  // Altitude in meters

  // Update shared data (protected by mutex)
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    roll = current_roll;
    pitch = current_pitch;
    yaw = current_yaw;
    altitude = current_altitude;
    xSemaphoreGive(xMutex);
  }
}

// Function to read GPS data
void readGPSData() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());  // Feed GPS data to TinyGPS++

    if (gps.location.isUpdated()) {
      // Update target position if not set
      if (target_latitude == 0 && target_longitude == 0) {
        target_latitude = gps.location.lat();
        target_longitude = gps.location.lng();
      }
    }
  }
}

// GPS Reading Task
void gpsTask(void *pvParameters) {
  while (1) {
    readGPSData();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100ms
  }
}

// Sensor Reading Task
void sensorTask(void *pvParameters) {
  while (1) {
    readSensorData();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10ms
  }
}

// PID Control Task
void pidTask(void *pvParameters) {
  unsigned long lastUpdateTime = millis();
  while (1) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;  // Calculate time difference
    lastUpdateTime = currentTime;

    // Read shared data (protected by mutex)
    float current_roll, current_pitch, current_yaw, current_altitude;
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      current_roll = roll;
      current_pitch = pitch;
      current_yaw = yaw;
      current_altitude = altitude;
      xSemaphoreGive(xMutex);
    }

    // Safety Check: Stop motors if drone is flipped or battery is low
    if (abs(current_roll) > SAFETY_ANGLE_THRESHOLD || abs(current_pitch) > SAFETY_ANGLE_THRESHOLD || battery_voltage < LOW_BATTERY_VOLTAGE) {
      setThrottle(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);  // Stop motors
      Serial.println("Safety triggered: Drone flipped or low battery. Motors stopped.");
      vTaskDelay(portMAX_DELAY);  // Halt the task
    }

    // Compute PID outputs for roll, pitch, yaw, and altitude
    float roll_output = computePID(-current_roll, roll_error_sum, last_roll_error, dt, Kp_roll, Ki_roll, Kd_roll);
    float pitch_output = computePID(-current_pitch, pitch_error_sum, last_pitch_error, dt, Kp_pitch, Ki_pitch, Kd_pitch);
    float yaw_output = computePID(-current_yaw, yaw_error_sum, last_yaw_error, dt, Kp_yaw, Ki_yaw, Kd_yaw);
    float altitude_error = TARGET_ALTITUDE - (current_altitude - ground_altitude);
    float altitude_output = computePID(altitude_error, altitude_error_sum, last_altitude_error, dt, Kp_altitude, Ki_altitude, Kd_altitude);

    // Position Hold using GPS
    if (gps.location.isValid()) {
      double current_latitude = gps.location.lat();
      double current_longitude = gps.location.lng();

      // Calculate position error (distance from target in meters)
      float position_error = TinyGPSPlus::distanceBetween(current_latitude, current_longitude, target_latitude, target_longitude);
      float position_output = computePID(position_error, position_error_sum, last_position_error, dt, Kp_position, Ki_position, Kd_position);

      // Adjust motor speeds based on position correction
      roll_output += position_output;
      pitch_output += position_output;
    }

    // Adjust motor speeds based on PID corrections
    int fl_adj = constrain(BASE_THROTTLE - roll_output - pitch_output + yaw_output + altitude_output, ESC_MIN, ESC_MAX);
    int fr_adj = constrain(BASE_THROTTLE + roll_output - pitch_output - yaw_output + altitude_output, ESC_MIN, ESC_MAX);
    int bl_adj = constrain(BASE_THROTTLE - roll_output + pitch_output - yaw_output + altitude_output, ESC_MIN, ESC_MAX);
    int br_adj = constrain(BASE_THROTTLE + roll_output + pitch_output + yaw_output + altitude_output, ESC_MIN, ESC_MAX);

    // Set motor speeds
    setThrottle(fl_adj, fr_adj, bl_adj, br_adj);

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10ms
  }
}

// State Management Task
void stateTask(void *pvParameters) {
  while (1) {
    switch (droneState) {
      case IDLE:
        // Calibrate ground altitude using VL53L0X
        ground_altitude = vl53.readRangeContinuousMillimeters() / 1000.0;
        droneState = TAKEOFF;
        takeoffStartTime = millis();
        Serial.println("Initiating takeoff...");
        break;

      case TAKEOFF: {
        float takeoffProgress = (float)(millis() - takeoffStartTime) / TAKEOFF_DURATION;
        takeoffProgress = constrain(takeoffProgress, 0.0, 1.0);  // Clamp between 0 and 1
        int takeoffThrottle = ESC_MIN + (BASE_THROTTLE - ESC_MIN) * takeoffProgress;

        // Set motor speeds
        setThrottle(takeoffThrottle, takeoffThrottle, takeoffThrottle, takeoffThrottle);

        // Check if takeoff is complete
        if (takeoffProgress >= 1.0) {
          droneState = HOVER;
          hoverStartTime = millis();
          Serial.println("Takeoff complete. Entering hover mode.");
        }
        break;
      }

      case HOVER:
        // Check if 10 seconds have passed
        if (millis() - hoverStartTime >= HOVER_DURATION) {
          droneState = LANDING;
          Serial.println("10 seconds of hover complete. Initiating landing...");
        }
        break;

      case LANDING: {
        float landingProgress = (float)(millis() - hoverStartTime) / TAKEOFF_DURATION;
        landingProgress = constrain(landingProgress, 0.0, 1.0);  // Clamp between 0 and 1
        int landingThrottle = BASE_THROTTLE - (BASE_THROTTLE - ESC_MIN) * landingProgress;

        // Set motor speeds
        setThrottle(landingThrottle, landingThrottle, landingThrottle, landingThrottle);

        // Check if landing is complete
        if (landingProgress >= 1.0) {
          droneState = STOP;
          Serial.println("Landing complete. Drone is idle.");
        }
        break;
      }

      case STOP:
        // Stop motors and halt the task
        setThrottle(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
        vTaskDelay(portMAX_DELAY);  // Halt the task
        break;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100ms
  }
}

void setup() {
  Serial.begin(115200);

  // Attach ESCs
  escFL.attach(MOTOR_FL, ESC_MIN, ESC_MAX);
  escFR.attach(MOTOR_FR, ESC_MIN, ESC_MAX);
  escBL.attach(MOTOR_BL, ESC_MIN, ESC_MAX);
  escBR.attach(MOTOR_BR, ESC_MIN, ESC_MAX);
  Serial.println("ESCs attached.");

  // Initialize I2C
  Wire.begin(19, 18);  // SDA = GPIO19, SCL = GPIO18
  Serial.println("I2C initialized.");

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);  // Halt if MPU6050 fails
  }
  Serial.println("MPU6050 initialized.");

  // Initialize BMP180 (not used for low-altitude control)
  if (!bmp.begin()) {
    Serial.println("BMP180 connection failed.");
    while (1);  // Halt if BMP180 fails
  }
  Serial.println("BMP180 initialized.");

  // Initialize QMC5883L
  compass.init();
  Serial.println("QMC5883L initialized.");

  // Initialize VL53L0X
  if (!vl53.init()) {
    Serial.println("VL53L0X connection failed.");
    while (1);  // Halt if VL53L0X fails
  }
  vl53.startContinuous();
  Serial.println("VL53L0X initialized.");

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // UART1 for GPS (TX = GPIO16, RX = GPIO17)
  Serial.println("GPS initialized.");

  // Arm ESCs
  setThrottle(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
  delay(5000);  // Wait for ESCs to arm
  Serial.println("ESCs armed.");

  // Create Mutex for shared data
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Failed to create mutex.");
    while (1);  // Halt if mutex creation fails
  }

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, &sensorTaskHandle, 1);  // Priority 2
  xTaskCreatePinnedToCore(pidTask, "PIDTask", 4096, NULL, 2, &pidTaskHandle, 1);           // Priority 2
  xTaskCreatePinnedToCore(stateTask, "StateTask", 4096, NULL, 1, &stateTaskHandle, 1);     // Priority 1
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, &gpsTaskHandle, 1);           // Priority 1

  Serial.println("FreeRTOS tasks created.");
}

void loop() {
  // FreeRTOS handles task scheduling, so loop() is empty
  vTaskDelete(NULL);  // Delete the loop task
}