#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <nrf.h>
#include <Nicla_System.h>

#include <sensors/SensorXYZ.h>
SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);

#include <sensors/Sensor.h>
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor temperature(BHY2_SENSOR_ID_TEMP);

#include <sensors/SensorQuaternion.h>
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);

#ifdef Mode
#undef Mode
#endif

#include <vl53l4cx_class.h>
VL53L4CX vl53l4cx(&Wire, NC);
VL53L4CX_UserRoi_t vl53l4cx_UserRoi = {6, 6, 9, 9};

#define SERIAL_BAUDRATE 115200
#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))
#define TCA9548A_ADDR 0x70
#define VL53L4CX_I2C_SPEED 400000
#define VL53L4CX_COUNT 8
#define CRITICAL_TASK_COUNT 3
#define BACKGROUND_TASK_COUNT 1
#define REQUEST_TYPE_COUNT 9
#define PA_TO_HPA 0.01f

#define ACCELEROMETER_RATE_HZ 400
#define ACCELEROMETER_LATENCY_MS 1
#define ACCELEROMETER_RANGE_G 8

#define GYROSCOPE_RATE_HZ 400
#define GYROSCOPE_LATENCY_MS 1
#define GYROSCOPE_RANGE_DPS 1000

#define MAGNETOMETER_RATE_HZ 20
#define MAGNETOMETER_LATENCY_MS 50
#define MAGNETOMETER_RANGE_UT 2500

#define HUMIDITY_RATE_HZ 1
#define HUMIDITY_LATENCY_MS 500

#define PRESSURE_RATE_HZ 1
#define PRESSURE_LATENCY_MS 500

#define TEMPERATURE_RATE_HZ 1
#define TEMPERATURE_LATENCY_MS 500

#define QUATERNION_RATE_HZ 400
#define QUATERNION_LATENCY_MS 1

#define YAW_LPF 0.75f
#define PITCH_LPF 0.75f
#define ROLL_LPF 0.75f
#define DISTANCE_LPF 0.75f

#define PRESSURE_LPF 0.25f
#define HUMIDITY_LPF 0.25f
#define TEMPERATURE_LPF 0.25f

#define YAW_OFFSET 0.0f
#define PITCH_OFFSET -3.0f
#define ROLL_OFFSET -1.0f
#define TEMP_OFFSET -7.0f

typedef struct __attribute__((packed, aligned(4)))
{
  float altitude;                    // meters (m)
  uint16_t distance[VL53L4CX_COUNT]; // millimeters (mm)
  float gps[2];                      // degrees [lat°, lon°]
  float humidity;                    // percent (%)
  float orientation[3];              // degrees [yaw°, pitch°, roll°]
  float pressure;                    // hectopascals (hPa)
  float speed;                       // meters per second (m/s)
  float temperature;                 // Celsius (°C)
} SensorValues_t;

static_assert(sizeof(SensorValues_t) == 56, "SensorValues_t struct size must be 56 bytes (14 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  const char *name;
  void (*task)(void);
  const uint32_t interval_us;
  uint32_t previous_us;
  uint32_t duration_us;
  uint32_t max_duration_us;
} Task_t;

static_assert(sizeof(Task_t) == 24, "Task_t struct size must be 24 bytes (6 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  const void *ptr;
  size_t size;
} SensorField_t;

static_assert(sizeof(SensorField_t) == 8, "SensorField_t struct must be 8 bytes (2 words)");

static inline void handle_tasks(void);
static inline void handle_serial(void);
static void vl53l4cx_select(const uint8_t channel);

static inline void task_imu_update(void);
static inline void task_gcu_update(void);
static inline void task_tof_update(void);

static Task_t critical_tasks[CRITICAL_TASK_COUNT] = {
    {"IMU", task_imu_update, HZ_TO_US(401), 0},
    {"GCU", task_gcu_update, HZ_TO_US(211), 0},
    {"TOF", task_tof_update, HZ_TO_US(31), 0},
};

static inline void task_dbg_update(void);

static Task_t background_tasks[BACKGROUND_TASK_COUNT] = {
    {"DBG", task_dbg_update, HZ_TO_US(1), 0},
};

static SensorValues_t response{
    .altitude = 0.0f,
    .distance = {0},
    .gps = {0.0f, 0.0f},
    .humidity = 50.0f,
    .orientation = {0.0f, 0.0f, 0.0f},
    .pressure = 1013.25f,
    .speed = 0.0f,
    .temperature = 25.0f};

static const SensorField_t handle_response[REQUEST_TYPE_COUNT] = {
    {&response, sizeof(SensorValues_t)},
    {&response.altitude, sizeof(response.altitude)},
    {&response.distance, sizeof(response.distance)},
    {&response.gps, sizeof(response.gps)},
    {&response.humidity, sizeof(response.humidity)},
    {&response.orientation, sizeof(response.orientation)},
    {&response.pressure, sizeof(response.pressure)},
    {&response.speed, sizeof(response.speed)},
    {&response.temperature, sizeof(response.temperature)}};

#endif // MAIN_H