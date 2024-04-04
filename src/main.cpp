#include "actuator.h"
#include <Arduino.h>

#include <FlexCAN_T4.h>
// clang-format off
#include <SPI.h>
// clang-format on
#include <HardwareSerial.h>
#include <SD.h>
#include <TimeLib.h>
#include <control_function_message.pb.h>
#include <header_message.pb.h>
#include <odrive.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>

using u8 = uint8_t;
using i8 = int8_t;
using u16 = uint16_t;
using i16 = int16_t;
using u32 = uint32_t;
using i32 = int32_t;
using u64 = uint64_t;
using i64 = int64_t;

enum class OperatingMode {
  Normal,
  Debug,
};
/**** Constants ****/
// Powertrain Constants
constexpr u8 engine_counts_per_rot = 16;
constexpr u8 gear_counts_per_rot = 6;

// NOTE: x_to_y ratio is how many rotations of x for 1 rotation of y
constexpr float gear_to_wheel_ratio = 1.0;
constexpr float secondary_to_wheel_ratio =
    ((46.0 / 17.0) * (56.0 / 19.0)); // ~7.975
constexpr float wheel_to_secondary_ratio =
    (1.0 / secondary_to_wheel_ratio);      // ~0.1253
constexpr float actuator_pitch = 5;        // 5 mm /rot
constexpr float actuator_range = 2.5;      // inches
constexpr float actuator_full_range = 3.5; // inches

// Unit Constants
constexpr float seconds_per_minute = 60.0;
constexpr float ms_per_second = 1.0e3;
constexpr float us_per_second = 1.0e6;
constexpr float seconds_per_ms = 1.0e-3;
constexpr float seconds_per_us = 1.0e-6;

// Control Function Constants
constexpr u32 control_function_interval_ms = 10;
constexpr float actuator_kp = 0.01;

// Pins
u8 engine_sensor_pin = 8;
u8 geartooth_sensor_pin = 9;

/**** Operation Flags ****/
constexpr OperatingMode operating_mode = OperatingMode::Normal;
constexpr bool wait_for_serial = false;

/**** Global Objects ****/
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive odrive(&flexcan_bus);
Actuator actuator(&odrive);

/**** Status Variables ****/
bool sd_initialized = false;

/**** ECVT State Variables ****/
// TODO: confirm variables only accessed inside timer ISR dont need to be
// volatile
u32 control_cycle_count = 0;

volatile u32 engine_count = 0;
volatile u32 gear_count = 0;
u32 last_engine_count = 0;
u32 last_gear_count = 0;
constexpr size_t log_buffer_size = 8192; // 12288 to be safe

// ave_write_size * frequency * flush_time * safety_factor
// 100 * 100 * 0.4 * 2 = 8000

struct LogBuffer {
  u8 buffer[log_buffer_size];
  size_t idx;
  bool full;
};

u8 cur_buffer_num = 0;
// TODO: can this be non-volatile assuming that there will be no overlap between
// ISR and loop
LogBuffer double_buffer[2];

// TODO profile serialization protobufs
// ඞ
void control_function() {
  LogBuffer *cur_buffer = &double_buffer[cur_buffer_num];
  float dt_s = control_function_interval_ms * seconds_per_ms;

  noInterrupts();
  u32 cur_engine_count = engine_count;
  u32 cur_gear_count = gear_count;
  interrupts();

  // Calculate instantaneous RPMs
  float engine_rpm = (cur_engine_count - last_engine_count) /
                     engine_counts_per_rot / dt_s * seconds_per_minute;
  float gear_rpm = (cur_gear_count - last_gear_count) / gear_counts_per_rot /
                   dt_s * seconds_per_minute;

  last_engine_count = cur_engine_count;
  last_gear_count = cur_gear_count;

  float wheel_rpm = gear_rpm * gear_to_wheel_ratio;
  float secondary_rpm = wheel_rpm * secondary_to_wheel_ratio;

  // TODO remove magic number
  float target_rpm = 3000.0;
  float error = target_rpm - engine_rpm;

  float velocity_command = error * actuator_kp;
  // actuator.set_speed(velocity_command);

  u8 log_message_data[512];

  size_t log_message_length = sizeof(log_message_data);

  if (cur_buffer->full) {
    Serial.print("Error: Something Very Bad is Happening!\n");
  } else if (cur_buffer->idx + log_message_length > log_buffer_size) {
    size_t remaining_space = log_buffer_size - cur_buffer->idx;
    memcpy(cur_buffer->buffer + cur_buffer->idx, log_message_data,
           remaining_space);
    cur_buffer->idx = log_buffer_size;
    cur_buffer->full = true;

    cur_buffer_num = !cur_buffer_num;
    cur_buffer = &double_buffer[cur_buffer_num];

    if (cur_buffer->idx != 0) {
      Serial.print("Error: Something Very Bad is Happening!\n");
    } else {
      memcpy(cur_buffer->buffer + cur_buffer->idx,
             log_message_data + remaining_space,
             log_message_length - remaining_space);
      cur_buffer->idx += log_message_length;
    }
  } else {
    memcpy(cur_buffer->buffer + cur_buffer->idx, log_message_data,
           log_message_length);
    cur_buffer->idx += log_message_length;
  }

  control_cycle_count++;
}

void debug_mode() {
  Serial.printf("Debug Mode: %d\n");
  control_cycle_count++;
}

File log_file;

void setup() {
  if (wait_for_serial) {
    while (!Serial) {
    }
  }

  // SD initialization
  sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
  if (!sd_initialized) {
    Serial.println("Warning: sd failed to initialize");
  }

  // log file determination and initialization
  // TODO skip log if SD failed?
  char log_name[64];
  snprintf(log_name, sizeof(log_name), "log_%04d-%02d-%02d_%02d-%02d-%02d.bin",
           year(), month(), day(), hour(), minute(), second());

  if (SD.exists(log_name)) {
    char log_name_duplicate[64];
    for (int log_num = 0; log_num < 1000; log_num++) {
      snprintf(log_name_duplicate, sizeof(log_name_duplicate), "%.*s_%03d.bin",
               23, log_name, log_num);
      if (!SD.exists(log_name_duplicate)) {
        break;
      }
    }
    strncpy(log_name, log_name_duplicate, sizeof(log_name));
    log_name[sizeof(log_name) - 1] = '\0';
  }
  log_file = SD.open(log_name, FILE_WRITE);

  attachInterrupt(
      engine_sensor_pin, []() { ++engine_count; }, FALLING);
  attachInterrupt(
      geartooth_sensor_pin, []() { ++gear_count; }, FALLING);

  switch (operating_mode) {
  case OperatingMode::Normal:
    timer.begin(control_function, control_function_interval_ms * 1e3);
    break;
  case OperatingMode::Debug:
    timer.begin(debug_mode, control_function_interval_ms * 1e3);
    break;
  }
}

void loop() {
  for (size_t buffer_num = 0; buffer_num < 2; buffer_num++) {
    if (double_buffer[buffer_num].full) {
      log_file.write(double_buffer[buffer_num].buffer, log_buffer_size);
      log_file.flush();
      double_buffer[buffer_num].full = false;
    }
  }
}
