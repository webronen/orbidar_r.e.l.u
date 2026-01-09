#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <nrf.h>
#include <nrf_delay.h>
#include <Wire.h>
#include <Nicla_System.h>

#include <sensors/Sensor.h>
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor temperature(BHY2_SENSOR_ID_TEMP);

#include <sensors/SensorQuaternion.h>
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);

#ifdef Mode
#undef Mode
#endif

#include <vl53l4cd_class.h>
VL53L4CD vl53l4cd(&Wire, -1);

#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define SEA_LEVEL_PRESSURE_HPA_INV (1.0f / SEA_LEVEL_PRESSURE_HPA)
#define ISA_ALT_SCALE_F 44330.76923077f
#define ISA_EXP_F 0.190263f

#define SERIAL_BAUDRATE 115200

#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))
#define MS_TO_US(ms) ((uint32_t)((ms) * 1000))
#define HZ_TO_MS(Hz) ((uint32_t)(1000.0f / (Hz)))
#define KHZ_TO_HZ(kHz) ((uint32_t)((kHz) * 1000))

#define PCF8574T_I2C_ADDRESS 0x20 // 7-bit address (8-bit is 0x40)
#define VL53L4CD_I2C_ADDRESS 0x52 // 8-bit address (7-bit is 0x29)
#define VL53L9CX_I2C_ADDRESS 0x52 // 8-bit address (7-bit is 0x29)

#define DISTANCE_I2C_ADDRESS 0x54 // Starting 8-bit address for VL53L4CD sensors (7-bit is 0x2A)

#define VL53L9CX_ZONE_WIDTH 54
#define VL53L9CX_ZONE_HEIGHT 42
#define VL53L9CX_ZONES (VL53L9CX_ZONE_WIDTH * VL53L9CX_ZONE_HEIGHT)
#define VL53L4CD_COUNT 7

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

#define CAMERA_LPF 0.75f
#define DISTANCE_LPF 0.75f
#define YAW_LPF 0.75f
#define PITCH_LPF 0.75f
#define ROLL_LPF 0.75f

#define HUMIDITY_LPF 0.25f
#define PRESSURE_LPF 0.25f
#define TEMPERATURE_LPF 0.25f

typedef struct __attribute__((packed, aligned(4)))
{
  float altitude;                    // meters (m)
  float humidity;                    // percentage (%)
  float orientation[3];              // degrees (°): [yaw, pitch, roll]
  float pressure;                    // hectopascal (hPa)
  float temperature;                 // degrees (°C)
  uint16_t camera[VL53L9CX_ZONES];   // millimeters (mm): 54 x 42 zones
  uint16_t distance[VL53L4CD_COUNT]; // millimeters (mm): 7 sensors
  uint8_t reserved[2];               // Padding for 4-byte alignment
} SensorValues_t;

static_assert(sizeof(SensorValues_t) == 4580, "SensorValues_t struct size must be 4580 bytes (1145 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  void (*task)(void);
  const uint32_t interval_us;
  uint32_t previous_us;
} Task_t;

static_assert(sizeof(Task_t) == 12, "Task_t struct size must be 12 bytes (3 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  const uint8_t *ptr;
  const uint16_t size;
  const uint8_t reserved[2]; // Padding for 4-byte alignment
} SensorField_t;

static_assert(sizeof(SensorField_t) == 8, "SensorField_t struct must be 8 bytes (2 words)");

static inline void handle_tasks(const uint32_t time);
static inline void handle_serial(void);
static inline void xshut_set(const int8_t pin, const bool level);
static inline void vl53l4cd_init(const uint8_t address, const uint8_t count);

static inline void sync_task_inertial(void);
static inline void sync_task_response(void);
static inline void sync_task_camera(void);

/**
 * @brief  Asynchronous ranging pipeline handler.
 *
 * This function implements a non-blocking measurement pipeline for multiple
 * VL53L4CD sensors. Each call performs two operations:
 *   1) Read and stop the sensor whose measurement was started during the previous call.
 *   2) Advance to the next sensor and start a new ranging operation.
 *
 * The index 'i' always refers to the sensor whose measurement is ready at the
 * beginning of the call. After reading and stopping that sensor, 'i' is
 * incremented (with wrap-around), and the next sensor is started.
 *
 * This creates a continuous pipeline where each sensor receives a full timing
 * budget between start and read, without blocking or polling.
 * 
 * @param  None
 * @return None
 * 
 * @note   Requires periodic execution (e.g. timer or scheduler) with a period
 *         greater than or equal to the configured timing budget.
 */
static inline void sync_task_distance(void);

static Task_t critical_tasks[SYNC_TASK_COUNT] = {
    {(void (*)())sync_task_inertial, (const uint32_t)HZ_TO_US(401)},
    {(void (*)())sync_task_response, (const uint32_t)HZ_TO_US(211)},
    {(void (*)())sync_task_camera, (const uint32_t)HZ_TO_US(61)},
    {(void (*)())sync_task_distance, (const uint32_t)HZ_TO_US(7)},
};

static inline void async_task_debug(void);

static Task_t background_tasks[ASYNC_TASK_COUNT] = {
    {(void (*)())async_task_debug, (const uint32_t)HZ_TO_US(1)},
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
    {(const uint8_t *)&response, (const uint16_t)sizeof(SensorValues_t)},                   // 0x00: Full response
    {(const uint8_t *)&response.altitude, (const uint16_t)sizeof(response.altitude)},       // 0x01: Altitude only
    {(const uint8_t *)&response.humidity, (const uint16_t)sizeof(response.humidity)},       // 0x02: Humidity only
    {(const uint8_t *)&response.orientation, (const uint16_t)sizeof(response.orientation)}, // 0x03: Orientation only
    {(const uint8_t *)&response.pressure, (const uint16_t)sizeof(response.pressure)},       // 0x04: Pressure only
    {(const uint8_t *)&response.temperature, (const uint16_t)sizeof(response.temperature)}, // 0x05: Temperature only
    {(const uint8_t *)&response.camera, (const uint16_t)sizeof(response.camera)},           // 0x06: Camera only
    {(const uint8_t *)&response.distance, (const uint16_t)sizeof(response.distance)},       // 0x07: Radial only
};

#endif // MAIN_H
