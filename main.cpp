#include "main.h"

void setup(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();

  sensortec.begin();
  accelerometer.begin(ACCELEROMETER_RATE_HZ, ACCELEROMETER_LATENCY_MS);
  accelerometer.setRange(ACCELEROMETER_RANGE_G);

  gyroscope.begin(GYROSCOPE_RATE_HZ, GYROSCOPE_LATENCY_MS);
  gyroscope.setRange(GYROSCOPE_RANGE_DPS);

  magnetometer.begin(MAGNETOMETER_RATE_HZ, MAGNETOMETER_LATENCY_MS);
  magnetometer.setRange(MAGNETOMETER_RANGE_UT);

  pressure.begin(PRESSURE_RATE_HZ, PRESSURE_LATENCY_MS);
  humidity.begin(HUMIDITY_RATE_HZ, HUMIDITY_LATENCY_MS);
  temperature.begin(TEMPERATURE_RATE_HZ, TEMPERATURE_LATENCY_MS);

  quaternion.begin(QUATERNION_RATE_HZ, QUATERNION_LATENCY_MS);

  // Wire.begin();
  // Wire.setClock(VL53L4CX_I2C_SPEED);

  // for (uint8_t i = 0; i < VL53L4CX_COUNT; i++)
  // {
  //   vl53l4cx_select(i);
  //   vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  //   vl53l4cx.VL53L4CX_WaitDeviceBooted();
  //   vl53l4cx.VL53L4CX_DataInit();
  //   vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  //   vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);
  //   vl53l4cx.VL53L4CX_SetUserROI(&vl53l4cx_UserRoi);
  //   vl53l4cx.VL53L4CX_StartMeasurement();
  // }

  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
    ;
}

void loop(void)
{
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t current_us = NRF_TIMER0->CC[0];

  handle_tasks(current_us);
  handle_serial();
}

static inline void handle_tasks(const uint32_t current_us)
{
  for (uint8_t i = 0; i < SYNC_TASK_COUNT; i++)
  {
    if ((int32_t)(current_us - critical_tasks[i].previous_us) >= 0)
    {
      critical_tasks[i].previous_us += critical_tasks[i].interval_us;
      critical_tasks[i].task();
    }
  }

  static uint8_t i = 0;
  if ((int32_t)(current_us - background_tasks[i].previous_us) >= (int32_t)background_tasks[i].interval_us)
  {
    background_tasks[i].previous_us = current_us;
    background_tasks[i].task();
  }

  i = (i + 1) % ASYNC_TASK_COUNT;
}

static inline void handle_serial(void)
{
  if (Serial.available() > 0)
  {
    const uint8_t type = (uint8_t)Serial.read() % REQUEST_TYPE_COUNT;
    Serial.write((const uint8_t *)handle_response[type].ptr, (size_t)handle_response[type].size);
  }
}

static void vl53l4cx_select(const uint8_t channel)
{
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write((uint8_t)(1 << channel));
  Wire.endTransmission();
}

static inline void task_imu_update(void)
{
  sensortec.update();
}

static inline void task_res_update(void)
{
  static bool calibrated = false;
  static uint16_t samples = 0;
  static DataQuaternion _q = {0.0f, 0.0f, 0.0f, 1.0f};

  if (!calibrated && ++samples >= 422)
  {
    _q.x = -quaternion._data.x;
    _q.y = -quaternion._data.y;
    _q.z = -quaternion._data.z;
    _q.w = quaternion._data.w;
    calibrated = true;
  }

  const DataQuaternion q = quaternion._data;

  float x = q.w * _q.x + q.x * _q.w + q.y * _q.z - q.z * _q.y;
  float y = q.w * _q.y - q.x * _q.z + q.y * _q.w + q.z * _q.x;
  float z = q.w * _q.z + q.x * _q.y - q.y * _q.x + q.z * _q.w;
  float w = q.w * _q.w - q.x * _q.x - q.y * _q.y - q.z * _q.z;

  const float mag = x * x + y * y + z * z + w * w;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  x *= inv;
  y *= inv;
  z *= inv;
  w *= inv;

  const float yaw = __builtin_atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
  const float roll = __builtin_asinf(2.0f * (w * y - x * z)) * RAD_TO_DEG;
  const float pitch = __builtin_atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;

  printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\r\n", yaw, pitch, roll);

  response.orientation[0] += (yaw - response.orientation[0]) * YAW_LPF;
  response.orientation[1] += (pitch - response.orientation[1]) * PITCH_LPF;
  response.orientation[2] += (roll - response.orientation[2]) * ROLL_LPF;

  response.pressure += (pressure._value - response.pressure) * PRESSURE_LPF;
  response.humidity += (humidity._value - response.humidity) * HUMIDITY_LPF;
  response.temperature += (temperature._value - response.temperature) * TEMPERATURE_LPF;
}

static inline void task_tof_update(void)
{
  static uint8_t ready;
  static VL53L4CX_MultiRangingData_t data;

  static uint8_t i = 0;
  vl53l4cx_select(i);

  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 &&
        data.RangeData[0].RangeStatus == 0)
    {
      response.distance[i] += (data.RangeData[0].RangeMilliMeter - response.distance[i]) * DISTANCE_LPF;
    }
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  i = (i + 1) % VL53L4CX_COUNT;
}

static inline void task_dbg_update(void)
{
  // Non critical debug updates can be placed here
}