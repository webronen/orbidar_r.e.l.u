#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <nrf.h>
#include <nrf_delay.h>

#include <Nicla_System.h>
#include <Wire.h>

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
VL53L4CX vl53l4cx(&Wire, -1);
VL53L4CX_UserRoi_t vl53l4cx_UserRoi = {6, 6, 9, 9}; // Centered 4x4 ROI

#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define SEA_LEVEL_PRESSURE_HPA_INV (1.0f / SEA_LEVEL_PRESSURE_HPA)
#define ISA_ALT_SCALE_F 44330.76923077f
#define ISA_EXP_F 0.190263f

#define SERIAL_BAUDRATE 115200
#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))
#define KHZ_TO_HZ(kHz) ((uint32_t)((kHz) * 1000))
#define VL53L9CX_ZONE_WIDTH 54
#define VL53L9CX_ZONE_HEIGHT 42
#define VL53L9CX_ZONES (VL53L9CX_ZONE_WIDTH * VL53L9CX_ZONE_HEIGHT)
#define VL53L4CX_COUNT 7
#define PCF8574_ADDRESS 0x20
#define VL53L4CX_COUNT 7

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
#define DIST_LPF 0.75f
#define YAW_LPF 0.75f
#define PITCH_LPF 0.75f
#define ROLL_LPF 0.75f

#define HUMIDITY_LPF 0.25f
#define PRESSURE_LPF 0.25f
#define TEMPERATURE_LPF 0.25f

typedef struct __attribute__((packet, aligned(4)))
{
  float altitude;                    // meters (m)
  float humidity;                    // percentage (%)
  float orientation[3];              // degrees (°): [yaw, pitch, roll]
  float pressure;                    // hectopascal (hPa)
  float temperature;                 // degrees (°C)
  uint16_t camera[VL53L9CX_ZONES];   // millimeters (mm): 54 x 42 zones
  uint16_t distance[VL53L4CX_COUNT]; // millimeters (mm): 7 sensors
  uint8_t reserved[2];               // Padding for 4-byte alignment
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
  const uint8_t *ptr;
  size_t size;
} SensorField_t;

static_assert(sizeof(SensorField_t) == 8, "SensorField_t struct must be 8 bytes (2 words)");

static const uint8_t vl53l4cx_address[7] = {
    0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60};

static inline void handle_tasks(const uint32_t current_us);
static inline void handle_serial(void);
static inline void xshut_set(const int8_t pin, const bool level);
static inline void vl53l4cx_init(void);

static inline void task_imu_update(void);
static inline void task_res_update(void);
static inline void task_cam_update(void);
static inline void task_dst_update(void);

static Task_t critical_tasks[SYNC_TASK_COUNT] = {
    {"IMU", task_imu_update, HZ_TO_US(401), 0},
    {"RES", task_res_update, HZ_TO_US(211), 0},
    {"CAM", task_cam_update, HZ_TO_US(61), 0},
    {"DST", task_dst_update, HZ_TO_US(5), 0},
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
    .distance = {0},
};

static const SensorField_t handle_response[REQUEST_TYPE_COUNT] = {
    {(const uint8_t *)&response, sizeof(SensorValues_t)},                   // 0x00: Full response
    {(const uint8_t *)&response.altitude, sizeof(response.altitude)},       // 0x01: Altitude only
    {(const uint8_t *)&response.humidity, sizeof(response.humidity)},       // 0x02: Humidity only
    {(const uint8_t *)&response.orientation, sizeof(response.orientation)}, // 0x03: Orientation only
    {(const uint8_t *)&response.pressure, sizeof(response.pressure)},       // 0x04: Pressure only
    {(const uint8_t *)&response.temperature, sizeof(response.temperature)}, // 0x05: Temperature only
    {(const uint8_t *)&response.camera, sizeof(response.camera)},           // 0x06: Camera only
    {(const uint8_t *)&response.distance, sizeof(response.distance)},       // 0x07: Radial only
};

#endif // MAIN_H
