#include "main.h"

void setup(void)
{
  nicla::begin(false);
  nicla::disableCharging();
  nicla::setBatteryNTCEnabled(false);
  nicla::disableLDO();

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
  xshut_set(NC, false);

  nicla::enable3V3LDO();
  nrf_delay_ms(100);

  vl53l4cx_init();

  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
    ;

  printf("I2C Scanner:\n");
  for (uint8_t i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    if (!Wire.endTransmission(true))
      printf("Found device at 0x%02X\n", i);
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
  const int type = Serial.read();
  if (type >= 0 && type < REQUEST_TYPE_COUNT)
    Serial.write(handle_response[type].ptr, handle_response[type].size);
}

static inline void xshut_set(const int8_t pin, const bool level)
{
  return;

  static uint8_t state = 0;
  state = level ? state | (1 << pin) : state & ~(1 << pin);

  Wire.beginTransmission(PCF8574_ADDRESS);
  Wire.write(pin < 0 ? 0x00 : state);
  Wire.endTransmission(true);
}

static inline void vl53l4cx_init(void)
{
  for (uint8_t i = 0; i < 1; i++)
  {
    xshut_set(i, HIGH);
    vl53l4cx.changeI2cAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS); // Every sensor boot to default address (0x52)
    vl53l4cx.VL53L4CX_WaitDeviceBooted();                       // Wait until the sensor is booted and in SW standby
    vl53l4cx.VL53L4CX_SetDeviceAddress(vl53l4cx_address[i]);    // Give the sensor a new unique address (ex. 0x54, 0x56, etc.)
    vl53l4cx.VL53L4CX_DataInit();                               // Initialize sensor to default settings (ex. distance mode, timing budget, etc.)
    vl53l4cx.VL53L4CX_SetUserROI(&vl53l4cx_UserRoi);            // Set user ROI (ex. 4x4 centered)
  }
}

static inline void sync_task_inertial(void)
{
  sensortec.update();
}

static inline void sync_task_response(void)
{
  DataQuaternion q = quaternion._data;

  const float mag = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
  q.w *= inv;

  const float yaw = __builtin_atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * RAD_TO_DEG;
  const float roll = __builtin_asinf(2.0f * (q.w * q.y - q.x * q.z)) * RAD_TO_DEG;
  const float pitch = __builtin_atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;

  response.orientation[0] += (yaw - response.orientation[0]) * YAW_LPF;
  response.orientation[1] += (pitch - response.orientation[1]) * PITCH_LPF;
  response.orientation[2] += (roll - response.orientation[2]) * ROLL_LPF;

  response.pressure += (pressure._value - response.pressure) * PRESSURE_LPF;
  response.humidity += (humidity._value - response.humidity) * HUMIDITY_LPF;
  response.temperature += (temperature._value - response.temperature) * TEMPERATURE_LPF;

  response.altitude = ISA_ALT_SCALE_F * (1.0f - __builtin_powf(response.pressure * SEA_LEVEL_PRESSURE_HPA_INV, ISA_EXP_F));
}

static inline void sync_task_camera(void)
{
  return;
}

static inline void sync_task_distance(void)
{
  static bool measurement_started = false;
  static uint8_t ready = 0;
  static VL53L4CX_MultiRangingData_t data;

  // ONE-TIME START: Only start measurement once
  if (!measurement_started)
  {
    vl53l4cx.changeI2cAddress(vl53l4cx_address[0]); // Use your sensor's address
    if (vl53l4cx.VL53L4CX_StartMeasurement() == VL53L4CX_ERROR_NONE)
    {
      measurement_started = true;
    }
    return; // Exit on this first call
  }

  // CONTINUOUS CHECK: Poll for data continuously
  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE)
    {
      if (data.NumberOfObjectsFound > 0 && data.RangeData[0].RangeStatus == 0)
      {
        // Use your LPF or direct assignment
        response.distance[0] = data.RangeData[0].RangeMilliMeter;
        // OR with your filter:
        // response.distance[0] += (data.RangeData[0].RangeMilliMeter - response.distance[0]) * DISTANCE_LPF;
      }
    }

    // CRITICAL: Clear interrupt and let sensor continue
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();

    // DO NOT CALL StopMeasurement() or change address here
  }
}

static inline void async_task_debug(void)
{
  // Debug distance measurements over Serial
  printf("Distances (mm): ");
  for (uint8_t i = 0; i < VL53L4CX_COUNT; i++)
  {
    printf("%u ", response.distance[i]);
  }
  printf("\n");
}