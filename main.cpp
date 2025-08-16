#include <Nicla_System.h>
#include <Wire.h>

#include <sensors/SensorXYZ.h>
#include <sensors/Sensor.h>
#include <sensors/SensorQuaternion.h>

#ifdef Mode
#undef Mode
#endif

#include <vl53l1_class.h>

#define REQUEST_ID_ALTITUDE 0x00
#define REQUEST_ID_DISTANCE 0x01
#define REQUEST_ID_GPS 0x02
#define REQUEST_ID_HUMIDITY 0x03
#define REQUEST_ID_ORIENTATION 0x04
#define REQUEST_ID_PRESSURE 0x05
#define REQUEST_ID_SPEED 0x06
#define REQUEST_ID_TEMPERATURE 0x07
#define REQUEST_ID_ALL 0xFF

#define PA_TO_HPA 0.01f
#define SEALEVEL_PRESSURE_HPA 1013.25f
#define ALTITUDE_CONSTANT 44330.0f
#define ALTITUDE_EXPONENT 0.1903f
#define EARTH_RADIUS_M 6371000.0f

#define ENV_LPF 0.1f
#define ENV_HPF 0.9f

#define DIST_LPF 0.2f
#define DIST_HPF 0.8f

#define SPEED_LPF 0.2f
#define SPEED_HPF 0.8f

#define CAM_LPF 0.2f
#define CAM_HPF 0.8f

#define SERIAL_BAUD 115200
#define TCA9548A_ADDR 0x70
#define VL53L1_ADDR 0x52
#define VL53L1_COUNT 8

static VL53L1 vl53l1(&Wire, NC);
static VL53L1_RoiConfig_t VL53L1_Roi = {
    .NumberOfRoi = 1,
    // Centered 4x4 ROI (Region of Interest)
    .UserRois = {
        {6, 6, 9, 9} // TopLeft (6, 6), BotRight (9, 9)
    }};

SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);

typedef struct
{
  float altitude;                  // meters (m)
  uint16_t distance[VL53L1_COUNT]; // millimeters (mm)
  float gps[2];                    // degrees [lat°, lon°]
  float humidity;                  // percent (%)
  float orientation[3];            // degrees [yaw°, pitch°, roll°]
  float pressure;                  // hectopascals (hPa)
  float speed;                     // meters per second (m/s)
  float temperature;               // Celsius (°C)
} SensorValues_t;

SensorValues_t sensorValues = {
    .altitude = 0.0f,
    .distance = {0},
    .gps = {0.0f, 0.0f},
    .humidity = 50.0f,
    .orientation = {0.0f, 0.0f, 0.0f},
    .pressure = SEALEVEL_PRESSURE_HPA,
    .speed = 0.0f,
    .temperature = 25.0f};

static void vl53l1_select(const uint8_t channel);
static inline void updateSensors(const uint32_t time_us);
static inline void handleRequest(const uint8_t requestType);

void setup(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP(); // Wait for HFCLK to start

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4; // 1us resolution (16 MHz / 2^4 = 1 MHz)
  NRF_TIMER0->TASKS_START = true;

  // Initialize sensors
  sensortec.begin();

  accelerometer.begin(100, 2); // 100 Hz sampling rate, 2 ms latency
  accelerometer.setRange(2);   // +/-2g

  gyroscope.begin(100, 2); // 100 Hz sampling rate, 2 ms latency
  gyroscope.setRange(250); // +/-250 dps

  magnetometer.begin(10, 10);  // 10 Hz sampling rate, 10 ms latency
  magnetometer.setRange(2500); // +/-2500 uT

  pressure.begin(2, 50);     // 2 Hz sampling rate, 50 ms latency
  humidity.begin(1, 100);    // 1 Hz sampling rate, 100 ms latency
  temperature.begin(1, 100); // 1 Hz sampling rate, 100 ms latency
  quaternion.begin(100, 2);  // 100 Hz sampling rate, 2 ms latency

  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

  // Initialize VL53L1 sensors
  for (uint8_t i = 0; i < VL53L1_COUNT; i++)
  {
    vl53l1_select(i); // Select the multiplexer channel (0-7)
    vl53l1.VL53L1_SetDeviceAddress(VL53L1_ADDR);
    vl53l1.VL53L1_WaitDeviceBooted();
    vl53l1.VL53L1_DataInit();
    vl53l1.VL53L1_StaticInit();
    vl53l1.VL53L1_SetDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
    vl53l1.VL53L1_SetMeasurementTimingBudgetMicroSeconds(33000);
    vl53l1.VL53L1_SetROI(&VL53L1_Roi);
    vl53l1.VL53L1_StartMeasurement();
  }

  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    __NOP(); // Wait for Serial to be ready
}

void loop(void)
{
  NRF_TIMER0->TASKS_CAPTURE[0] = true;
  const uint32_t time_us = NRF_TIMER0->CC[0];

  updateSensors(time_us);

  if (Serial.available() > 0)
  {
    const uint8_t requestType = Serial.read();
    handleRequest(requestType);
  }
}

static void vl53l1_select(const uint8_t channel)
{
  const uint8_t mask = (1 << channel);

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(mask);
  Wire.endTransmission();
}

static inline void updateSensors(const uint32_t time_us)
{
  sensortec.update();

  // EMA (Exponential Moving Average) filter for accelerometer, gyroscope, and magnetometer
  sensorValues.pressure = __builtin_fmaf(ENV_LPF, pressure._value * PA_TO_HPA, ENV_HPF * sensorValues.pressure);
  sensorValues.humidity = __builtin_fmaf(ENV_LPF, humidity._value, ENV_HPF * sensorValues.humidity);
  sensorValues.temperature = __builtin_fmaf(ENV_LPF, temperature._value, ENV_HPF * sensorValues.temperature);

  // Calculate altitude based on pressure using the barometric formula
  sensorValues.altitude = ALTITUDE_CONSTANT * (1.0f - __builtin_powf(sensorValues.pressure / SEALEVEL_PRESSURE_HPA, ALTITUDE_EXPONENT));

  float x = quaternion._data.x;
  float y = quaternion._data.y;
  float z = quaternion._data.z;
  float w = quaternion._data.w;

  // Normalize the quaternion
  const float norm_sq = x * x + y * y + z * z + w * w;
  const float inv_norm = 1.0f / __builtin_sqrtf(norm_sq + __FLT_EPSILON__); // Epsilon to avoid division by zero

  x *= inv_norm;
  y *= inv_norm;
  z *= inv_norm;
  w *= inv_norm;

  // Convert quaternion to Euler angles (yaw, pitch, roll)
  sensorValues.orientation[0] = __builtin_atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG; // Yaw
  sensorValues.orientation[1] = __builtin_asinf(2.0f * (w * y - x * z)) * RAD_TO_DEG;                                 // Pitch
  sensorValues.orientation[2] = __builtin_atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG; // Roll

  for (uint8_t i = 0; i < VL53L1_COUNT; i++)
  {
    vl53l1_select(i); // Select the multiplexer channel

    uint8_t ready = 0;
    if (vl53l1.VL53L1_GetMeasurementDataReady(&ready) == VL53L1_ERROR_NONE && ready)
    {
      // Data is ready and no error, safe to read measurement
      VL53L1_MultiRangingData_t data;
      if (vl53l1.VL53L1_GetMultiRangingData(&data) == VL53L1_ERROR_NONE &&
          data.NumberOfObjectsFound > 0 && data.RangeData[0].RangeStatus == 0)
      {
        // Data is valid, apply EMA filter
        const float distance = __builtin_fmaf(DIST_LPF, (float)data.RangeData[0].RangeMilliMeter, DIST_HPF * (float)sensorValues.distance[i]);
        sensorValues.distance[i] = (uint16_t)distance;
      }

      vl53l1.VL53L1_ClearInterruptAndStartMeasurement();
    }
  }
}

// Handle serial requests
static inline void handleRequest(const uint8_t requestType)
{
  switch (requestType)
  {
  case REQUEST_ID_ALTITUDE:
    Serial.write((const uint8_t *)&sensorValues.altitude, sizeof(float));
    break;
  case REQUEST_ID_DISTANCE:
    Serial.write((const uint8_t *)sensorValues.distance, sizeof(uint16_t) * VL53L1_COUNT);
    break;
  case REQUEST_ID_GPS:
    Serial.write((const uint8_t *)sensorValues.gps, sizeof(float) * 2);
    break;
  case REQUEST_ID_HUMIDITY:
    Serial.write((const uint8_t *)&sensorValues.humidity, sizeof(float));
    break;
  case REQUEST_ID_ORIENTATION:
    Serial.write((const uint8_t *)sensorValues.orientation, sizeof(float) * 3);
    break;
  case REQUEST_ID_PRESSURE:
    Serial.write((const uint8_t *)&sensorValues.pressure, sizeof(float));
    break;
  case REQUEST_ID_TEMPERATURE:
    Serial.write((const uint8_t *)&sensorValues.temperature, sizeof(float));
    break;
  case REQUEST_ID_SPEED:
    Serial.write((const uint8_t *)&sensorValues.speed, sizeof(float));
    break;
  case REQUEST_ID_ALL:
    Serial.write((const uint8_t *)&sensorValues, sizeof(SensorValues_t));
    break;
  default:
    break;
  }
}
