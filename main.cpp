#include "main.h"

void setup(void)
{
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

  // Wire.begin();
  // Wire.setClock(KHZ_TO_HZ(400));

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
    {
      Serial.write((const uint8_t *)handle_response[type].ptr, (size_t)handle_response[type].size);
      Serial.flush();
    }
  }
}

static inline void cam_init(void)
{
  return;
}

static inline void tof_init(void)
{
  return;
}

static inline void task_imu_update(void)
{
  sensortec.update();
}

static inline void task_res_update(void)
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

static inline void task_tof_update(void)
{
  return;
}

static inline void task_dbg_update(void)
{
  return;
}
