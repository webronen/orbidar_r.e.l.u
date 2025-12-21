#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <nrf.h>

// #include <sensors/SensorXYZ.h>
// SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
// SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
// SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);

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

#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define SEA_LEVEL_PRESSURE_HPA_INV (1.0f / SEA_LEVEL_PRESSURE_HPA)
#define ISA_ALT_SCALE_F 44330.76923077f
#define ISA_EXP_F 0.190263f

#define SERIAL_BAUDRATE 115200
#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))
#define KHZ_TO_HZ(kHz) ((uint32_t)((kHz) * 1000))

#define TOF_COUNT 7
#define CAM_COUNT 1
#define CAM_WIDTH 54
#define CAM_HEIGHT 42
#define SYNC_TASK_COUNT 4
#define ASYNC_TASK_COUNT 1
#define REQUEST_TYPE_COUNT 8

#define HUMIDITY_RATE_HZ 2
#define HUMIDITY_LATENCY_MS 1000

#define PRESSURE_RATE_HZ 2
#define PRESSURE_LATENCY_MS 1000

#define TEMPERATURE_RATE_HZ 2
#define TEMPERATURE_LATENCY_MS 1000

#define QUATERNION_RATE_HZ 400
#define QUATERNION_LATENCY_MS 1

#define CAM_LPF 0.75f
#define TOF_LPF 0.75f
#define YAW_LPF 0.75f
#define PITCH_LPF 0.75f
#define ROLL_LPF 0.75f

#define HUMIDITY_LPF 0.25f
#define PRESSURE_LPF 0.25f
#define TEMPERATURE_LPF 0.25f

typedef struct __attribute__((packet, aligned(4)))
{
  float altitude;                          // meters (m)
  float humidity;                          // percent (%)
  float orientation[3];                    // degrees [yaw°, pitch°, roll°]
  float pressure;                          // hectopascals (hPa)
  float temperature;                       // Celsius (°C)
  uint16_t camera[CAM_WIDTH * CAM_HEIGHT]; // 2D depth map from VL53L9CX
  uint16_t radial[TOF_COUNT];              // 1D depth array from VL53L4CX
  uint8_t reserved[2];                     // Padding for 4-byte alignment
} SensorValues_t;

static_assert(sizeof(SensorValues_t) == 4580, "SensorValues_t struct size must be 4580 bytes (1145 words)");

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

static const uint8_t xshutPins[2] = {20, 9};

static inline void handle_tasks(const uint32_t current_us);
static inline void handle_serial(void);
static inline void cam_init(void);
static inline void tof_init(void);

static inline void task_imu_update(void);
static inline void task_res_update(void);
static inline void task_cam_update(void);
static inline void task_tof_update(void);

static Task_t critical_tasks[SYNC_TASK_COUNT] = {
    {"IMU", task_imu_update, HZ_TO_US(401), 0},
    {"RES", task_res_update, HZ_TO_US(211), 0},
    {"CAM", task_cam_update, HZ_TO_US(61), 0},
    {"TOF", task_tof_update, HZ_TO_US(31), 0},
};

static inline void task_dbg_update(void);

static Task_t background_tasks[ASYNC_TASK_COUNT] = {
    {"DBG", task_dbg_update, HZ_TO_US(1), 0},
};

static SensorValues_t response{
    .altitude = 0.0f,
    .humidity = 50.0f,
    .orientation = {0.0f, 0.0f, 0.0f},
    .pressure = 1013.25f,
    .temperature = 25.0f,
    .camera = {0},
    .radial = {0},
};

static const SensorField_t handle_response[REQUEST_TYPE_COUNT] = {
    {(const uint8_t *)&response, sizeof(SensorValues_t)},                           // 0x00: Full response
    {(const uint8_t *)&response.altitude, sizeof(response.altitude)},               // 0x01: Altitude only
    {(const uint8_t *)&response.humidity, sizeof(response.humidity)},               // 0x02: Humidity only
    {(const uint8_t *)&response.orientation, 3 * sizeof(float)},                    // 0x03: Orientation only
    {(const uint8_t *)&response.pressure, sizeof(response.pressure)},               // 0x04: Pressure only
    {(const uint8_t *)&response.temperature, sizeof(response.temperature)},         // 0x05: Temperature only
    {(const uint8_t *)&response.camera, CAM_WIDTH * CAM_HEIGHT * sizeof(uint16_t)}, // 0x06: Camera only
    {(const uint8_t *)&response.radial, TOF_COUNT * sizeof(uint16_t)},              // 0x07: Radial only
};

#endif // MAIN_H
