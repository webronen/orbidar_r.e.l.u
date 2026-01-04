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

  xshut_set(-1, false); // Ensure all XSHUT pins are LOW before initialization
  nrf_delay_ms(100);    // Wait all sensors to power down

  vl53l4cd_init(0x54, VL53L4CD_COUNT); // Initialize VL53L4CD sensors

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
  static uint8_t state = 0x00; // Current state of all XSHUT pins
  // If pin < 0, set all pins LOW, else set specific pin to LOW or HIGH
  state = pin < 0 ? 0x00 : (level ? (state | (1 << pin)) : (state & ~(1 << pin)));
  Wire.master->write(PCF8574T_I2C_ADDRESS << 1, (const char *)&state, 1, true);
}

static inline void vl53l4cd_init(const uint8_t address, const uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    xshut_set(i, true);
    nrf_delay_ms(100);

    vl53l4cd.dev = VL53L4CD_I2C_ADDRESS;
    vl53l4cd.VL53L4CD_SensorInit();
    vl53l4cd.VL53L4CD_SetI2CAddress(address + i * 2);
  }
}

static inline void sync_task_inertial(void)
{
  sensortec.update();
}

static inline void sync_task_response(void)
{
  DataQuaternion q = quaternion._data;

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

  vl53l4cd.dev = DISTANCE_I2C_ADDRESS + j * 2;

  VL53L4CD_ResultData_t result = {0};
  if (!vl53l4cd.VL53L4CD_GetResultData(&result) &&
      result.range_status == 9 &&
      result.distance_mm >= 0 &&
      result.distance_mm <= 1300)
  {
    response.distance[j] += (result.distance_mm - response.distance[j]) * DISTANCE_LPF;
  }

  vl53l4cd.VL53L4CD_ClearInterrupt();
  vl53l4cd.VL53L4CD_StopRanging();

  i = (i + 1) % VL53L4CD_COUNT;
  vl53l4cd.dev = DISTANCE_I2C_ADDRESS + i * 2;
  vl53l4cd.VL53L4CD_StartRanging();
}

static inline void async_task_debug(void)
{
  printf("Altitude: %.2f m, Pressure: %.2f hPa, Humidity: %.2f %%, Temperature: %.2f °C\n",
         response.altitude,
         response.pressure,
         response.humidity,
         response.temperature);

  printf("Orientation: Yaw: %.2f °, Pitch: %.2f °, Roll: %.2f °\n",
         response.orientation[0],
         response.orientation[1],
         response.orientation[2]);

  printf("Distances (mm): ");
  for (uint8_t i = 0; i < VL53L4CD_COUNT; i++)
    printf("%u ", response.distance[i]);

  printf("\n");
}