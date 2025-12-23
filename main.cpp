#include "main.h"

void setup(void)
{
  nicla::begin(false);
  nicla::disableCharging();
  nicla::setBatteryNTCEnabled(false);
  nicla::disableLDO();
  nrf_delay_ms(100);

  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  sensortec.begin();
  pressure.begin(PRESSURE_RATE_HZ, PRESSURE_LATENCY_MS);
  humidity.begin(HUMIDITY_RATE_HZ, HUMIDITY_LATENCY_MS);
  temperature.begin(TEMPERATURE_RATE_HZ, TEMPERATURE_LATENCY_MS);
  quaternion.begin(QUATERNION_RATE_HZ, QUATERNION_LATENCY_MS);

  Wire.begin();
  Wire.setClock(KHZ_TO_HZ(400));
  xshut_set(NC, false); // Disable all ToF sensors

  nicla::enable3V3LDO();
  nrf_delay_ms(100);

  vl53l4cx_init();

  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
    ;

  printf("I2C Scanner:\n");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (!Wire.endTransmission(true))
    {
      printf("Found device at 0x%02X\n", addr);
    }
  }
  printf("Scan complete.\n");
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
  // Handle synchronous (critical) tasks
  for (uint8_t i = 0; i < SYNC_TASK_COUNT; i++)
  {
    if ((int32_t)(current_us - critical_tasks[i].previous_us) >= 0)
    {
      critical_tasks[i].previous_us += critical_tasks[i].interval_us;
      critical_tasks[i].task();
    }
  }

  // Handle asynchronous (background) tasks
  static uint8_t i = 0;
  if ((int32_t)(current_us - background_tasks[i].previous_us) >= (int32_t)background_tasks[i].interval_us)
  {
    background_tasks[i].previous_us = current_us;
    background_tasks[i].task();
  }

  i = (i + 1) % ASYNC_TASK_COUNT; // Move to the next background task for the next call (round-robin scheduling)
}

static inline void handle_serial(void)
{
  if (Serial.available() > 0)
  {
    const int type = Serial.read();
    if (type >= 0 && type < REQUEST_TYPE_COUNT)
      Serial.write(handle_response[type].ptr, handle_response[type].size);
  }
}

static inline void xshut_set(int8_t pin, bool level)
{
  static uint8_t state = 0;
  state = level ? state | (1 << pin) : state & ~(1 << pin);

  Wire.beginTransmission(PCF8574_ADDRESS);
  Wire.write(pin < 0 ? 0x00 : state);
  Wire.endTransmission(true);
}

static inline void vl53l4cx_init(void)
{
  for (uint8_t sensor = 0; sensor < VL53L4CX_COUNT; sensor++)
  {
    xshut_set(sensor, HIGH);                                      // Enable current sensor
    vl53l4cx.changeI2cAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);   // Set default I2C address
    vl53l4cx.VL53L4CX_WaitDeviceBooted();                         // Wait for sensor to boot
    vl53l4cx.VL53L4CX_SetDeviceAddress(vl53l4cx_address[sensor]); // Set unique I2C address
    vl53l4cx.VL53L4CX_WaitDeviceBooted();                         // Wait for sensor to reboot with new address
    vl53l4cx.VL53L4CX_DataInit();                                 // Initialize sensor (Distance mode medium, 33ms timing budget)
    vl53l4cx.VL53L4CX_SetUserROI(&vl53l4cx_UserRoi);              // Set centered 4x4 ROI
  }
}

static inline void task_imu_update(void)
{
  sensortec.update();
}

static inline void task_res_update(void)
{
  DataQuaternion q = quaternion._data;

  // Normalize quaternion
  const float mag = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
  q.w *= inv;

  // Convert quaternion to Euler angles (yaw, pitch, roll) in degrees
  const float yaw = __builtin_atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
  const float roll = __builtin_asinf(2.0f * (q.w * q.y - q.x * q.z)) * RAD_TO_DEG;
  const float pitch = __builtin_atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;

  // Apply EMA filtering to orientation and environmental data
  response.orientation[0] += (yaw - response.orientation[0]) * YAW_LPF;
  response.orientation[1] += (pitch - response.orientation[1]) * PITCH_LPF;
  response.orientation[2] += (roll - response.orientation[2]) * ROLL_LPF;

  response.pressure += (pressure._value - response.pressure) * PRESSURE_LPF;
  response.humidity += (humidity._value - response.humidity) * HUMIDITY_LPF;
  response.temperature += (temperature._value - response.temperature) * TEMPERATURE_LPF;

  // Calculate altitude using the barometric formula (ISA model, valid up to 11km)
  response.altitude = ISA_ALT_SCALE_F * (1.0f - __builtin_powf(response.pressure * SEA_LEVEL_PRESSURE_HPA_INV, ISA_EXP_F));
}

static inline void task_cam_update(void)
{
  return;
}

static inline void task_dst_update(void)
{
  static bool measuring = false;
  static uint8_t sensor = 0;

  if (!measuring)
  {
    vl53l4cx.changeI2cAddress(vl53l4cx_address[sensor]);
    measuring = !vl53l4cx.VL53L4CX_StartMeasurement();
    return;
  }

  uint8_t ready = 0;
  if (!vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) && ready)
  {
    VL53L4CX_MultiRangingData_t data;
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 &&
        data.RangeData[0].RangeStatus == 0)
    {
      response.distance[sensor] += (data.RangeData[0].RangeMilliMeter - response.distance[sensor]) * DIST_LPF;
      vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
      vl53l4cx.VL53L4CX_StopMeasurement();
      measuring = false;
      sensor = (sensor + 1) % VL53L4CX_COUNT;
    }
  }
}

static inline void task_dbg_update(void)
{
  return;
}
