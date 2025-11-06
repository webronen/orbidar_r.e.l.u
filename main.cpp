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

  Wire.begin();
  Wire.setClock(VL53L4CX_I2C_SPEED);

  for (uint8_t i = 0; i < VL53L4CX_COUNT; i++)
  {
    vl53l4cx_select(i); // Select the multiplexer channel

    vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
    vl53l4cx.VL53L4CX_WaitDeviceBooted();
    vl53l4cx.VL53L4CX_DataInit();
    vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
    vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);

    VL53L4CX_UserRoi_t roi = {6, 6, 9, 9}; // Set ROI to 4x4 centered
    vl53l4cx.VL53L4CX_SetUserROI(&roi);
    vl53l4cx.VL53L4CX_StartMeasurement();
  }

  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
    ;
}

void loop(void)
{
  // Capture start timer value in microseconds into CC[0]
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  // Handle scheduled tasks
  handle_tasks();
  // Handle incoming serial requests
  handle_serial();
  // Capture end timer value in microseconds into CC[1]
  NRF_TIMER0->TASKS_CAPTURE[1] = 1;
  // Calculate total loop duration and store in CC[3] for profiling/monitoring
  NRF_TIMER0->CC[3] = NRF_TIMER0->CC[1] - NRF_TIMER0->CC[0];
}

/**
 * Hybrid task scheduler
 *
 * Design philosophy:
 * ------------------
 * - Critical (hard real-time) tasks:
 *   * Checked every loop iteration.
 *   * Use strict periodicity (previous_us += interval_us).
 *   * Guarantees deterministic timing and evenly spaced execution.
 *   * Suitable for control loops, sensor sampling, or comms deadlines.
 *
 * - Background (soft) tasks:
 *   * One task is checked per loop (round-robin).
 *   * Use best-effort resync (previous_us = current_us).
 *   * Prevents backlog/catch-up bursts if the system is delayed.
 *   * Suitable for logging, housekeeping, or non-critical work.
 *
 * This separation ensures:
 *   * Predictability and safety for time-sensitive operations.
 *   * Smooth CPU usage and scalability for non-critical tasks.
 *   * Energy efficiency: CPU can idle/sleep once critical work is done.
 */
static inline void handle_tasks(void)
{
  // Capture current timestamp once at the start of the loop
  const uint32_t current_us = NRF_TIMER0->CC[0];

  // -------------------------------------------------------------------------
  // Handle critical (hard real-time) tasks
  // -------------------------------------------------------------------------
  for (uint8_t j = 0; j < CRITICAL_TASK_COUNT; j++)
  {
    // Check if this task is due (wraparound-safe comparison).
    if ((int32_t)(current_us - critical_tasks[j].previous_us) >= 0)
    {
      // Advance schedule by fixed interval (strict periodicity).
      // Even if delayed, the next run stays aligned to the original timeline.
      critical_tasks[j].previous_us += critical_tasks[j].interval_us;

      // Measure execution time using TIMER capture.
      NRF_TIMER0->TASKS_CAPTURE[1] = 1;
      critical_tasks[j].task();
      NRF_TIMER0->TASKS_CAPTURE[2] = 1;

      // Store duration for profiling/monitoring.
      critical_tasks[j].duration_us = NRF_TIMER0->CC[2] - NRF_TIMER0->CC[1];

      // Update max execution time if this run exceeded previous max.
      if (critical_tasks[j].duration_us > critical_tasks[j].max_duration_us)
        critical_tasks[j].max_duration_us = critical_tasks[j].duration_us;
    }
  }

  // -------------------------------------------------------------------------
  // Handle background (soft) tasks
  // -------------------------------------------------------------------------
  static uint8_t i = 0; // round-robin index across background tasks

  // Check if the current background task is due
  if ((int32_t)(current_us - background_tasks[i].previous_us) >=
      (int32_t)background_tasks[i].interval_us)
  {
    // Resync to "now" (best-effort scheduling).
    // Prevents multiple back-to-back runs if the system was delayed.
    background_tasks[i].previous_us = current_us;

    // Measure execution time for this background task.
    NRF_TIMER0->TASKS_CAPTURE[1] = 1;
    background_tasks[i].task();
    NRF_TIMER0->TASKS_CAPTURE[2] = 1;

    background_tasks[i].duration_us = NRF_TIMER0->CC[2] - NRF_TIMER0->CC[1];

    if (background_tasks[i].duration_us > background_tasks[i].max_duration_us)
      background_tasks[i].max_duration_us = background_tasks[i].duration_us;
  }

  // Advance round-robin index (wrap around at BACKGROUND_TASK_COUNT).
  i = (i + 1) % BACKGROUND_TASK_COUNT;
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
  const uint8_t mask = (1 << (channel % VL53L4CX_COUNT));

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(mask);
  Wire.endTransmission();
}

static inline void task_imu_update(void)
{
  sensortec.update();

  float x = quaternion._data.x;
  float y = quaternion._data.y;
  float z = quaternion._data.z;
  float w = quaternion._data.w;

  // Calculate quaternion magnitude and normalize. Add epsilon to avoid division by zero.
  const float mag = x * x + y * y + z * z + w * w;
  const float norm = __builtin_sqrtf(mag + __FLT_EPSILON__);

  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  // Convert normalized quaternion to Euler angles (yaw, pitch, roll) in degrees. Pitch is shifted by +90° because of sensor mounting orientation.
  const float yaw = __builtin_atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
  const float pitch = (__builtin_atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG) + 90.0f;
  const float roll = __builtin_asinf(2.0f * (w * y - x * z)) * RAD_TO_DEG;

  /**
   * Apply low-pass filtering (LPF) to smooth out sensor noise and rapid fluctuations.
   * The EMA/IIR formula used is:
   *   old_value += (new_value - old_value) * alpha_factor
   * alpha_factor is 0 <= alpha_factor <= 1
   *
   * A higher alpha_factor means less smoothing (faster response).
   * A lower alpha_factor means more smoothing (slower response).
   *
   * This helps stabilize the orientation readings for applications like UI control or motion detection.
   * 
   * TODO: Add offsets/calibration for real-world alignment.
   */
  response.orientation[0] += (response.orientation[0] - yaw) * YAW_LPF;
  response.orientation[1] += (response.orientation[1] - pitch) * PITCH_LPF;
  response.orientation[2] += (response.orientation[2] - roll) * ROLL_LPF;

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

  i = (i + 1) % VL53L4CX_COUNT; // Advance to next sensor index for next call (round-robin)
}

static inline void task_dbg_update(void)
{
  const uint32_t loop_time_us = NRF_TIMER0->CC[3];
  float loop_hz = (loop_time_us > 0)
                      ? (1000000.0f / loop_time_us)
                      : 0.0f;

  printf("\n=== Scheduler Debug ===\n");
  if (loop_time_us == 0)
  {
    printf("Loop Time: 0 us (N/A Hz)\n");
  }
  else
  {
    printf("Loop Time: %lu us (%.2f Hz)\n", loop_time_us, loop_hz);
  }

  printf("-------------------------------------------------------------------------------------\n");
  printf("| Type       | ID | Exec(us) | Max(us) | Rate(Hz) | Load(%%) | Margin(%%) | Slack(us) |\n");
  printf("-------------------------------------------------------------------------------------\n");

  // Critical tasks
  for (uint8_t i = 0; i < CRITICAL_TASK_COUNT; i++)
  {
    uint32_t exec = critical_tasks[i].duration_us;
    uint32_t max = critical_tasks[i].max_duration_us;
    uint32_t period = critical_tasks[i].interval_us;

    float hz = (period > 0) ? (1000000.0f / period) : 0.0f;
    float load = (period > 0) ? (exec * 100.0f) / period : 0.0f;
    float margin = (period > 0) ? 100.0f - ((max * 100.0f) / period) : 0.0f;
    int32_t slack = (period > exec) ? (period - exec) : 0;

    printf("| Critical   | %2u | %8lu | %7lu | %8.2f | %7.2f | %8.2f | %9ld  |\n",
           i, exec, max, hz, load, margin, (long)slack);
  }

  // Background tasks
  for (uint8_t i = 0; i < BACKGROUND_TASK_COUNT; i++)
  {
    uint32_t exec = background_tasks[i].duration_us;
    uint32_t max = background_tasks[i].max_duration_us;
    uint32_t period = background_tasks[i].interval_us;

    float hz = (period > 0) ? (1000000.0f / period) : 0.0f;
    float load = (period > 0) ? (exec * 100.0f) / period : 0.0f;
    float margin = (period > 0) ? 100.0f - ((max * 100.0f) / period) : 0.0f;
    int32_t slack = (period > exec) ? (period - exec) : 0;

    printf("| Background | %2u | %8lu | %7lu | %8.2f | %7.2f | %8.2f | %9ld  |\n",
           i, exec, max, hz, load, margin, (long)slack);
  }

  printf("-------------------------------------------------------------------------------------\n");
}