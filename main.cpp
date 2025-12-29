#include "main.h"

void setup(void)
{
  nicla::begin(false);
  nicla::disableCharging();
  nicla::setBatteryNTCEnabled(false);
  nicla::enable3V3LDO();

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
  nrf_delay_ms(400);
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

static inline void i2c_switch(const int8_t channel)
{
  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  Wire.write((channel < 0) ? 0 : (1 << channel));
  Wire.endTransmission(true);
}

static inline void vl53l4cd_init(void)
{
  for (uint8_t i = 0; i < VL53L4CD_COUNT; i++)
  {
    i2c_switch(i);
    vl53l4cd.VL53L4CD_SensorInit();
  }
}

static inline void sync_task_inertial(void)
{
  sensortec.update();
}

static inline void sync_task_response(void)
{
  static uint8_t samples = 0;
  static DataQuaternion _q;

  DataQuaternion q = quaternion._data;

  if (++samples <= 211)
  {
    _q.x += (q.x - _q.x) * (1.0f / samples);
    _q.y += (q.y - _q.y) * (1.0f / samples);
    _q.z += (q.z - _q.z) * (1.0f / samples);
    _q.w += (q.w - _q.w) * (1.0f / samples);

    return;
  }

  q.x -= _q.x;
  q.y -= _q.y;
  q.z -= _q.z;
  q.w -= _q.w;

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
  static uint8_t i = 0;
  const uint8_t j = (i - 1 + VL53L4CD_COUNT) % VL53L4CD_COUNT;

  i2c_switch(j);

  uint16_t distance = 0;
  if (!vl53l4cd.VL53L4CD_GetDistance(&distance))
    response.distance[j] += (distance - response.distance[j]) * DISTANCE_LPF;

  vl53l4cd.VL53L4CD_StopRanging();

  i2c_switch(i);
  vl53l4cd.VL53L4CD_StartRanging();

  i = (i + 1) % VL53L4CD_COUNT;
}

static inline void async_task_debug(void)
{
  // Debug distance measurements over Serial
  printf("Distances (mm): ");
  for (uint8_t i = 0; i < VL53L4CD_COUNT; i++)
  {
    printf("%u ", response.distance[i]);
  }
  printf("\n");
}