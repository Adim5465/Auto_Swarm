#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP085.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_mesh.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_netif.h>

// Sensor Objects
QMC5883LCompass compass;
VL53L0X vl53;
TinyGPSPlus gps;
MPU6050 mpu;
Adafruit_BMP085 bmp;
ESP32Servo esc1, esc2, esc3, esc4;

// MAVLink and ESP-MESH Configuration
uint8_t MAVLINK_SYSTEM_ID = 1;
#define MAVLINK_COMPONENT_ID 1    
#define MAVLINK_SERIAL Serial2    
#define MAVLINK_BAUDRATE 57600
#define MESH_CHANNEL 6
#define MESH_ROUTER_SSID "DRONE_SWARM"
#define MESH_ROUTER_PASS "swarmpassword"
#define MESH_PORT 5555

// Motor GPIO Pins
#define MOTOR1_PIN 25
#define MOTOR2_PIN 26
#define MOTOR3_PIN 27
#define MOTOR4_PIN 14

// ESC Settings
#define ESC_MIN 1000
#define BASE_THROTTLE 1500
#define ESC_MAX 2000

// Data Structures
typedef struct {
  uint8_t drone_id;
  float latitude;
  float longitude;
  float altitude;
  float heading;
  uint8_t role; // 0=leader, 1=follower
} swarm_position_t;

typedef struct {
  uint8_t drone_id;
  uint8_t command; // 0=halt, 1=takeoff, 2=land, 3=formation
  float param1;
  float param2;
  float param3;
} swarm_command_t;

// Global Variables
swarm_position_t my_position = {0};
swarm_position_t swarm_positions[10];
uint8_t swarm_size = 0;
bool is_leader = false;
bool mesh_connected = false;
uint8_t leader_id = 0;

float TARGET_ALTITUDE = 5.0; // Default target altitude (meters)
double target_latitude = 0.0;
double target_longitude = 0.0;
float ground_altitude = 0.0;
unsigned long takeoffStartTime = 0;
unsigned long hoverStartTime = 0;
#define TAKEOFF_DURATION 5000

enum DroneState { IDLE, TAKEOFF, HOVER, LANDING, STOP };
DroneState droneState = IDLE;

// Task Handles
TaskHandle_t gpsTaskHandle = nullptr;
TaskHandle_t stateTaskHandle = nullptr;
TaskHandle_t mavlinkTaskHandle = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t pidTaskHandle = nullptr;
TaskHandle_t meshTaskHandle = nullptr;

// Mutex
SemaphoreHandle_t xMutex = nullptr;

// Function Declarations
void mesh_receive_cb(const mesh_addr_t *from, const mesh_data_t *data);
void meshTask(void *pvParameters);
void stateTask(void *pvParameters);
void mavlinkTask(void *pvParameters);
void sensorTask(void *pvParameters);
void pidTask(void *pvParameters);
void gpsTask(void *pvParameters);
void init_mesh_network();
void send_swarm_position();
void send_swarm_command(uint8_t command, float param1 = 0, float param2 = 0, float param3 = 0);
void elect_leader();
void formation_control();
void setMotorSpeeds(int speed);

// Mesh Event Handler
static void mesh_event_handler(void *arg, esp_event_base_t event_base, 
                             int32_t event_id, void *event_data) {
  switch (event_id) {
    case MESH_EVENT_STARTED:
      Serial.println("MESH_EVENT_STARTED");
      break;
      
    case MESH_EVENT_STOPPED:
      Serial.println("MESH_EVENT_STOPPED");
      mesh_connected = false;
      break;
      
    case MESH_EVENT_CHANNEL_SWITCH:
      Serial.println("MESH_EVENT_CHANNEL_SWITCH");
      break;
      
    case MESH_EVENT_PARENT_CONNECTED:
      Serial.println("MESH_EVENT_PARENT_CONNECTED");
      mesh_connected = true;
      if (is_leader) {
        Serial.println("This drone is the swarm leader");
      }
      break;
      
    case MESH_EVENT_NO_PARENT_FOUND:
      Serial.println("MESH_EVENT_NO_PARENT_FOUND");
      break;
      
    case MESH_EVENT_LAYER_CHANGE:
      Serial.println("MESH_EVENT_LAYER_CHANGE");
      break;
      
    default:
      break;
  }
}

// Mesh Receive Callback
void mesh_receive_cb(const mesh_addr_t *from, const mesh_data_t *data) {
  if (data->size == sizeof(swarm_position_t)) {
    swarm_position_t *pos = (swarm_position_t *)data->data;
    
    bool found = false;
    for (int i = 0; i < swarm_size; i++) {
      if (swarm_positions[i].drone_id == pos->drone_id) {
        memcpy(&swarm_positions[i], pos, sizeof(swarm_position_t));
        found = true;
        break;
      }
    }
    
    if (!found && swarm_size < 10) {
      memcpy(&swarm_positions[swarm_size++], pos, sizeof(swarm_position_t));
    }
  } else if (data->size == sizeof(swarm_command_t)) {
    swarm_command_t *cmd = (swarm_command_t *)data->data;
    
    if (cmd->drone_id == leader_id && !is_leader) {
      switch (cmd->command) {
        case 0: droneState = STOP; break;
        case 1: 
          TARGET_ALTITUDE = cmd->param1;
          droneState = TAKEOFF;
          takeoffStartTime = millis();
          break;
        case 2: droneState = LANDING; break;
        case 3: break; // Formation command
      }
    }
  }
}

// Mesh Initialization
void init_mesh_network() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
  ESP_ERROR_CHECK(esp_wifi_start());
  
  mesh_cfg_t mesh_cfg = {
    .channel = MESH_CHANNEL,
    .mesh_ap = {
      .max_connection = 6,
    }
  };
  strncpy((char *)mesh_cfg.router.ssid, MESH_ROUTER_SSID, sizeof(mesh_cfg.router.ssid));
  strncpy((char *)mesh_cfg.router.password, MESH_ROUTER_PASS, sizeof(mesh_cfg.router.password));
  strncpy((char *)mesh_cfg.mesh_ap.password, MESH_ROUTER_PASS, sizeof(mesh_cfg.mesh_ap.password));
  
  ESP_ERROR_CHECK(esp_mesh_init());
  ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_cfg));
  
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  MAVLINK_SYSTEM_ID = mac[5];
  my_position.drone_id = MAVLINK_SYSTEM_ID;
  
  ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
  ESP_ERROR_CHECK(esp_mesh_set_rx_cb(mesh_receive_cb));
  
  ESP_ERROR_CHECK(esp_mesh_start());
}

// Swarm Functions
void send_swarm_position() {
  if (mesh_connected) {
    my_position.latitude = gps.location.isValid() ? gps.location.lat() : 0;
    my_position.longitude = gps.location.isValid() ? gps.location.lng() : 0;
    my_position.altitude = !vl53.timeoutOccurred() ? vl53.readRangeContinuousMillimeters() / 1000.0 : 0;
    
    compass.read();
    my_position.heading = compass.getAzimuth();
    my_position.role = is_leader ? 0 : 1;
    
    mesh_data_t mesh_data = {
      .data = (uint8_t *)&my_position,
      .size = sizeof(my_position),
      .proto = MESH_PROTO_BIN,
      .tos = MESH_TOS_P2P
    };
    ESP_ERROR_CHECK(esp_mesh_send(NULL, &mesh_data, MESH_DATA_P2P, NULL, 0));
  }
}

void send_swarm_command(uint8_t command, float param1, float param2, float param3) {
  if (is_leader && mesh_connected) {
    swarm_command_t cmd = {
      .drone_id = my_position.drone_id,
      .command = command,
      .param1 = param1,
      .param2 = param2,
      .param3 = param3
    };
    
    mesh_data_t mesh_data = {
      .data = (uint8_t *)&cmd,
      .size = sizeof(cmd),
      .proto = MESH_PROTO_BIN,
      .tos = MESH_TOS_P2P
    };
    ESP_ERROR_CHECK(esp_mesh_send(NULL, &mesh_data, MESH_DATA_P2P, NULL, 0));
  }
}

void elect_leader() {
  uint8_t min_id = my_position.drone_id;
  for (int i = 0; i < swarm_size; i++) {
    if (swarm_positions[i].drone_id < min_id) {
      min_id = swarm_positions[i].drone_id;
    }
  }
  is_leader = (min_id == my_position.drone_id);
  leader_id = min_id;
}

void formation_control() {
  if (!is_leader && swarm_size > 0) {
    for (int i = 0; i < swarm_size; i++) {
      if (swarm_positions[i].drone_id == leader_id) {
        float distance = 2.0;
        float angle = radians(swarm_positions[i].heading);
        target_latitude = swarm_positions[i].latitude + (distance * sin(angle)) / 111320.0;
        target_longitude = swarm_positions[i].longitude + (distance * cos(angle)) / (111320.0 * cos(radians(swarm_positions[i].latitude)));
        TARGET_ALTITUDE = swarm_positions[i].altitude;
        break;
      }
    }
  }
}

// Motor Control
void setMotorSpeeds(int speed) {
  speed = constrain(speed, ESC_MIN, ESC_MAX);
  esc1.writeMicroseconds(speed);
  esc2.writeMicroseconds(speed);
  esc3.writeMicroseconds(speed);
  esc4.writeMicroseconds(speed);
}

// Tasks
void stateTask(void *pvParameters) {
  while (1) {
    switch (droneState) {
      case IDLE:
        if (!vl53.timeoutOccurred()) {
          ground_altitude = vl53.readRangeContinuousMillimeters() / 1000.0;
        }
        if (is_leader) {
          send_swarm_command(1, TARGET_ALTITUDE);
          droneState = TAKEOFF;
          takeoffStartTime = millis();
        }
        break;

      case TAKEOFF: {
        float takeoffProgress = (float)(millis() - takeoffStartTime) / TAKEOFF_DURATION;
        takeoffProgress = constrain(takeoffProgress, 0.0, 1.0);
        int takeoffThrottle = ESC_MIN + (BASE_THROTTLE - ESC_MIN) * takeoffProgress;
        setMotorSpeeds(takeoffThrottle);
        
        if (takeoffProgress >= 1.0) {
          droneState = HOVER;
          hoverStartTime = millis();
          if (is_leader) {
            send_swarm_command(3, 1); // Formation command
          }
        }
        break;
      }

      case HOVER:
        if (mesh_connected) {
          formation_control();
          setMotorSpeeds(BASE_THROTTLE);
        }
        break;

      case LANDING: {
        float landingProgress = (float)(millis() - hoverStartTime) / TAKEOFF_DURATION;
        landingProgress = constrain(landingProgress, 0.0, 1.0);
        int landingThrottle = BASE_THROTTLE - (BASE_THROTTLE - ESC_MIN) * landingProgress;
        setMotorSpeeds(landingThrottle);
        
        if (landingProgress >= 1.0) {
          droneState = STOP;
          if (is_leader) {
            send_swarm_command(0); // Halt command
          }
        }
        break;
      }

      case STOP:
        setMotorSpeeds(ESC_MIN);
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void meshTask(void *pvParameters) {
  init_mesh_network();
  
  while (1) {
    if (mesh_connected) {
      send_swarm_position();
      
      static unsigned long last_election = 0;
      if (millis() - last_election > 5000) {
        elect_leader();
        last_election = millis();
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void mavlinkTask(void *pvParameters) {
  while (1) {
    if (MAVLINK_SERIAL.available()) {
      uint8_t byte = MAVLINK_SERIAL.read();
      // Process MAVLink messages here
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sensorTask(void *pvParameters) {
  // Initialize sensors
  Wire.begin();
  compass.init();
  compass.setCalibration(-1767, 1355, -1961, 714, -1738, 1566);
  
  if (!vl53.init()) {
    Serial.println("Failed to initialize VL53L0X!");
    while(1);
  }
  vl53.setTimeout(500);
  vl53.startContinuous();

  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 sensor!");
    while(1);
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  while (1) {
    // Read and process sensor data
    compass.read();
    vl53.readRangeContinuousMillimeters();
    bmp.readPressure();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Get accelerometer and gyro data
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void pidTask(void *pvParameters) {
  // PID controller implementation
  while (1) {
    // Add PID control logic here
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void gpsTask(void *pvParameters) {
  HardwareSerial gpsSerial(1); // Use UART1 for GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  while (1) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize ESCs
  esc1.attach(MOTOR1_PIN, ESC_MIN, ESC_MAX);
  esc2.attach(MOTOR2_PIN, ESC_MIN, ESC_MAX);
  esc3.attach(MOTOR3_PIN, ESC_MIN, ESC_MAX);
  esc4.attach(MOTOR4_PIN, ESC_MIN, ESC_MAX);
  setMotorSpeeds(ESC_MIN);
  delay(2000); // Wait for ESCs to initialize

  // Create mutex
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Mutex creation failed!");
    while(1);
  }

  // Create tasks
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 3, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(pidTask, "PIDTask", 4096, NULL, 4, &pidTaskHandle, 1);
  xTaskCreatePinnedToCore(stateTask, "StateTask", 4096, NULL, 2, &stateTaskHandle, 1);
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(mavlinkTask, "MAVLinkTask", 4096, NULL, 1, &mavlinkTaskHandle, 1);
  xTaskCreatePinnedToCore(meshTask, "MeshTask", 8192, NULL, 1, &meshTaskHandle, 1);

  Serial.println("Drone initialization complete");
}

void loop() {
  vTaskDelete(NULL); // FreeRTOS tasks handle the operation
}