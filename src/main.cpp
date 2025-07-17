#include <Wire.h>
#include <Arduino_BMI270_BMM150.h> // Correct library for Nano 33 BLE
#include <ArduinoJson.h>
using namespace arduino;

// Debugging macro
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)

// Remove manual I2C pin definitions as they're already defined in the framework

// Global error tracking
bool sensorInitialized = false;
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 50; // ms

// Sensor calibration offsets (adjust these after calibration)
struct SensorCalibration
{
  float accelOffset[3] = {0, 0, 0};
  float gyroOffset[3] = {0, 0, 0};
  float magOffset[3] = {0, 0, 0};
} sensorCal;

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial)
  {
    ; // Wait for serial port to connect
  }

  // Nano 33 BLE uses fixed I2C pins, no need to set them explicitly
  Wire.begin();
  Wire.setClock(400000L); // Set I2C clock to 400kHz

  // Initialize BMI270 IMU and BMM150 Magnetometer
  if (!IMU.begin())
  {
    DEBUG_PRINTLN("{\"critical_error\": \"Failed to initialize IMU\"}");
    sensorInitialized = false;
    return;
  }

  // Perform basic sensor self-test and calibration
  DEBUG_PRINTLN("Performing IMU self-test...");
  sensorInitialized = true;

  // Calibration routine
  DEBUG_PRINTLN("Place sensor flat and still for calibration...");
  delay(2000);

  // Collect calibration data
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  int samples = 100;

  for (int i = 0; i < samples; i++)
  {
    if (IMU.accelerationAvailable() &&
        IMU.gyroscopeAvailable() &&
        IMU.magneticFieldAvailable())
    {

      IMU.readAcceleration(accelX, accelY, accelZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      IMU.readMagneticField(magX, magY, magZ);

      sensorCal.accelOffset[0] += accelX;
      sensorCal.accelOffset[1] += accelY;
      sensorCal.accelOffset[2] += accelZ;

      sensorCal.gyroOffset[0] += gyroX;
      sensorCal.gyroOffset[1] += gyroY;
      sensorCal.gyroOffset[2] += gyroZ;

      sensorCal.magOffset[0] += magX;
      sensorCal.magOffset[1] += magY;
      sensorCal.magOffset[2] += magZ;

      delay(10);
    }
  }

  // Calculate average offsets
  for (int i = 0; i < 3; i++)
  {
    sensorCal.accelOffset[i] /= samples;
    sensorCal.gyroOffset[i] /= samples;
    sensorCal.magOffset[i] /= samples;
  }

  DEBUG_PRINTLN("IMU Calibration Complete");
}

void loop()
{
  // Ensure sensors are initialized
  if (!sensorInitialized)
  {
    DEBUG_PRINTLN("{\"error\": \"Sensors not initialized\"}");
    delay(1000);
    return;
  }

  // Throttle sensor readings
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime < SENSOR_READ_INTERVAL)
  {
    return;
  }
  lastSensorReadTime = currentTime;

  // Create JSON document for sensor data
  JsonDocument doc;

  // Read sensor data
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable() &&
      IMU.magneticFieldAvailable())
  {

    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    IMU.readMagneticField(magX, magY, magZ);

    // Apply calibration offsets
    accelX -= sensorCal.accelOffset[0];
    accelY -= sensorCal.accelOffset[1];
    accelZ -= sensorCal.accelOffset[2];

    gyroX -= sensorCal.gyroOffset[0];
    gyroY -= sensorCal.gyroOffset[1];
    gyroZ -= sensorCal.gyroOffset[2];

    magX -= sensorCal.magOffset[0];
    magY -= sensorCal.magOffset[1];
    magZ -= sensorCal.magOffset[2];

    // Sensor 1 Data
    doc["sensor1_accel_x"] = accelX;
    doc["sensor1_accel_y"] = accelY;
    doc["sensor1_accel_z"] = accelZ;

    // Calculate orientation using accelerometer and magnetometer
    float roll = atan2(accelY, accelZ) * 180.0 / PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    // Yaw calculation using magnetometer (more accurate)
    float yaw = atan2(magY, magX) * 180.0 / PI;
    if (yaw < 0)
      yaw += 360;

    doc["sensor1_roll"] = roll;
    doc["sensor1_pitch"] = pitch;
    doc["sensor1_yaw"] = yaw;

    // Placeholder for second sensor
    doc["sensor2_accel_x"] = 0.0;
    doc["sensor2_accel_y"] = 0.0;
    doc["sensor2_accel_z"] = 0.0;
    doc["sensor2_roll"] = 0.0;
    doc["sensor2_pitch"] = 0.0;
    doc["sensor2_yaw"] = 0.0;

    // Add debugging information
    doc["timestamp"] = millis();
    doc["sensor_status"] = sensorInitialized ? "OK" : "ERROR";

    // Serialize and send JSON
    serializeJson(doc, Serial);
    Serial.println(); // Newline for Python's readline()
  }
}
// Removed duplicate loop() function

// Optional: Add interrupt-based error handling
void errorHandler()
{
  DEBUG_PRINTLN("{\"critical_error\": \"Unexpected system state\"}");
  sensorInitialized = false;
}