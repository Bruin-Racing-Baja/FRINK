#include "core_pins.h"
#include <Arduino.h>
#include <actuator.h>
#include <cmath>
#include <constants.h>

#include <FlexCAN_T4.h>
// clang-format off
#include <SPI.h>
// clang-format on
#include <HardwareSerial.h>
#include <SD.h>
#include <TimeLib.h>
#include <control_function_message.pb.h>
#include <cstring>
#include <header_message.pb.h>
#include <odrive.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <stddef.h>
#include <stdio.h>
#include <types.h>

enum class OperatingMode {
  NORMAL,
  DEBUG,
};

/**** Operation Flags ****/
constexpr OperatingMode operating_mode = OperatingMode::NORMAL;
constexpr bool wait_for_serial = true;

/**** Global Objects ****/
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive odrive(&flexcan_bus);
Actuator actuator(&odrive);
File log_file;

/**** Status Variables ****/
bool sd_initialized = false;

/**** ECVT State Variables ****/
// TODO: Confirm variables only accessed in timer ISR dont need to be volatile
u32 control_cycle_count = 0;

volatile u32 engine_count = 0;
volatile u32 gear_count = 0;
u32 last_engine_count = 0;
u32 last_gear_count = 0;

/**** Logging Variables ****/
struct LogBuffer {
  char buffer[LOG_BUFFER_SIZE];
  size_t idx;
  bool full;
};

// TODO: Can these be non-volatile assuming no overlap between
// ISR and loop (should be no overlap if buffer is implemented correctly)
u8 cur_buffer_num = 0;
LogBuffer double_buffer[2];

/**** Global Functions ****/
time_t get_teensy3_time() { return Teensy3Clock.get(); }

void can_parse(const CAN_message_t &msg) { odrive.parse_message(msg); }

// ඞ
void control_function() {
  u32 cycle_start_us = micros();
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;

  // Grab sensor data
  noInterrupts();
  u32 cur_engine_count = engine_count;
  u32 cur_gear_count = gear_count;
  interrupts();

  // Calculate instantaneous RPMs
  float engine_rpm = (cur_engine_count - last_engine_count) /
                     ENGINE_COUNTS_PER_ROT / dt_s * SECONDS_PER_MINUTE;
  float gear_rpm = (cur_gear_count - last_gear_count) / GEAR_COUNTS_PER_ROT /
                   dt_s * SECONDS_PER_MINUTE;

  last_engine_count = cur_engine_count;
  last_gear_count = cur_gear_count;

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  float secondary_rpm = wheel_rpm * SECONDARY_TO_WHEEL_RATIO;

  // Controller
  float target_rpm = ENGINE_TARGET_RPM;
  float error = target_rpm - engine_rpm;

  float velocity_command = error * ACTUATOR_KP;
  // actuator.set_velocity(velocity_command);

  // TODO: Use protobuf for storing system state
  ControlFunctionMessage log_message = ControlFunctionMessage_init_default;
  log_message.secondary_rpm = secondary_rpm;
  log_message.engine_rpm = engine_rpm;
  log_message.cycle_count = control_cycle_count;
  log_message.cycle_start_us = cycle_start_us;
  log_message.engine_count = cur_engine_count;
  log_message.gear_count = cur_gear_count;
  log_message.engine_rpm = engine_rpm;
  log_message.secondary_rpm = secondary_rpm;
  log_message.target_rpm = target_rpm;
  log_message.engine_rpm_error = error;
  log_message.velocity_command = velocity_command;

  // TODO: Profile serialization
  u8 log_message_data[ControlFunctionMessage_size + PROTO_DELIMITER_LENGTH];
  pb_ostream_t ostream =
      pb_ostream_from_buffer(log_message_data + PROTO_DELIMITER_LENGTH,
                             sizeof(log_message_data) - PROTO_DELIMITER_LENGTH);
  pb_encode(&ostream, &ControlFunctionMessage_msg, &log_message);

  size_t log_message_length = ostream.bytes_written;

  char prefix[PROTO_DELIMITER_LENGTH + 1];
  snprintf(prefix, PROTO_DELIMITER_LENGTH + 1, "%01X%04X",
           PROTO_CONTROL_FUNCTION_MESSAGE_ID, log_message_length);
  memcpy(log_message_data, prefix, PROTO_DELIMITER_LENGTH);

  log_message_length += PROTO_DELIMITER_LENGTH;

  LogBuffer *cur_buffer = &double_buffer[cur_buffer_num];

  if (cur_buffer->full) {
    Serial.print("Error: BAD! Current buffer is full :(\n");
  } else if (cur_buffer->idx + log_message_length > LOG_BUFFER_SIZE) {
    size_t remaining_space = LOG_BUFFER_SIZE - cur_buffer->idx;
    memcpy(cur_buffer->buffer + cur_buffer->idx, log_message_data,
           remaining_space);
    cur_buffer->idx = LOG_BUFFER_SIZE;
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
  noInterrupts();
  u32 cur_engine_count = engine_count;
  u32 cur_gear_count = gear_count;
  interrupts();
  Serial.printf("debug: %d, %d\n", cur_engine_count, cur_gear_count);
  // Serial.printf("Debug Mode: %d, %d, %d\n", control_cycle_count,
  // cur_engine_count, cur_gear_count);
  control_cycle_count++;
}

void setup() {
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN2_LED_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);

  for (size_t i = 0; i < sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  pinMode(ENGINE_SENSOR_PIN, INPUT);
  pinMode(GEARTOOTH_SENSOR_PIN, INPUT);

  pinMode(THROTTLE_POT_PIN, INPUT);
  pinMode(BRAKE_SENSOR_PIN, INPUT);

  pinMode(LIMIT_SWITCH_IN_PIN, INPUT);
  pinMode(LIMIT_SWITCH_OUT_PIN, INPUT);
  pinMode(LIMIT_SWITCH_ENGAGE_PIN, INPUT);

  if (wait_for_serial) {
    while (!Serial) {
    }
  }

  setSyncProvider(get_teensy3_time);

  if (timeStatus() != timeSet) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  // SD initialization
  sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
  if (!sd_initialized) {
    Serial.println("Warning: SD failed to initialize");
  }

  // TODO: Skip log if SD failed
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
      ENGINE_SENSOR_PIN, []() { ++engine_count; }, FALLING);
  attachInterrupt(
      GEARTOOTH_SENSOR_PIN, []() { ++gear_count; }, FALLING);

  flexcan_bus.begin();
  flexcan_bus.setBaudRate(FLEXCAN_BAUD_RATE);
  flexcan_bus.setMaxMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(can_parse);

  // TODO: Figure out proper ISR priority levels
  NVIC_SET_PRIORITY(IRQ_CAN1, 1);

  switch (operating_mode) {
  case OperatingMode::NORMAL:
    timer.begin(control_function, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::DEBUG:
    timer.begin(debug_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  }
}

void loop() {
  for (size_t buffer_num = 0; buffer_num < 2; buffer_num++) {
    if (double_buffer[buffer_num].full) {
      Serial.printf("Flush: Writing buffer %d to SD\n", buffer_num);
      log_file.write(double_buffer[buffer_num].buffer, LOG_BUFFER_SIZE);
      log_file.flush();
      double_buffer[buffer_num].full = false;
      double_buffer[buffer_num].idx = 0;
    }
  }
}
