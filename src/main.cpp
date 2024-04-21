#include <Arduino.h>
#include <actuator.h>
#include <constants.h>

#include <FlexCAN_T4.h>
// clang-format off
#include <SPI.h>
// clang-format on
#include <HardwareSerial.h>
#include <SD.h>
#include <TimeLib.h>
#include <control_function_state.pb.h>
#include <cstring>
#include <odrive.h>
#include <operation_header.pb.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <stddef.h>
#include <stdio.h>
#include <types.h>

// Acknowledgements to Getty & Tyler :)

enum class OperatingMode {
  NORMAL,
  BUTTON_SHIFT,
  DEBUG,
  NONE,
};

/**** Operation Flags ****/
constexpr OperatingMode operating_mode = OperatingMode::NORMAL;
constexpr bool wait_for_serial = true;

/**** Global Objects ****/
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive odrive(&flexcan_bus, ODRIVE_NODE_ID);
Actuator actuator(&odrive);
File log_file;

/**** Status Variables ****/
bool sd_initialized = false;

/**** ECVT State Variables ****/
// TODO: Confirm variables only accessed in timer ISR dont need to be volatile
u32 control_cycle_count = 0;

volatile u32 engine_count = 0;
u32 last_engine_count = 0;

volatile u32 gear_count = 0;
volatile u32 gear_time_diff_us = 0;
u32 last_gear_time_us = 0;

ControlFunctionState control_state = ControlFunctionState_init_default;

/**** System State Variables ****/
bool last_button_state[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

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

u8 message_buffer[MESSAGE_BUFFER_SIZE];

/**** Global Functions ****/

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))

time_t get_teensy3_time() { return Teensy3Clock.get(); }

void can_parse(const CAN_message_t &msg) { odrive.parse_message(msg); }

inline void write_all_leds(u8 state) {
  digitalWrite(GREEN_LED_PIN, state);
  digitalWrite(YELLOW_LED_PIN, state);
  digitalWrite(RED_LED_PIN, state);
  digitalWrite(GREEN2_LED_PIN, state);
  digitalWrite(WHITE_LED_PIN, state);
}

size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                         const pb_msgdesc_t *fields,
                         const void *message_struct) {
  // Serialize message
  pb_ostream_t ostream = pb_ostream_from_buffer(
      buffer + PROTO_DELIMITER_LENGTH, buffer_length - PROTO_DELIMITER_LENGTH);
  pb_encode(&ostream, fields, message_struct);

  size_t message_length = ostream.bytes_written;

  // Create message delimiter
  char delimiter[PROTO_DELIMITER_LENGTH + 1];
  snprintf(delimiter, PROTO_DELIMITER_LENGTH + 1, "%01X%04X", id,
           message_length);
  memcpy(buffer, delimiter, PROTO_DELIMITER_LENGTH);
  message_length += PROTO_DELIMITER_LENGTH;

  return message_length;
}

constexpr u8 DOUBLE_BUFFER_SUCCESS = 0;
constexpr u8 DOUBLE_BUFFER_FULL_ERROR = 1;
constexpr u8 DOUBLE_BUFFER_INDEX_ERROR = 2;

u8 write_to_double_buffer(u8 data[], size_t data_length,
                          LogBuffer double_buffer[2], u8 *buffer_num,
                          bool split) {
  LogBuffer *cur_buffer = &double_buffer[*buffer_num];

  if (cur_buffer->full) {
    // If current buffer is full then something is wrong
    return DOUBLE_BUFFER_FULL_ERROR;
  } else if (cur_buffer->idx + data_length > LOG_BUFFER_SIZE) {
    // If data_length exceeds remaining space in buffer

    size_t remaining_space = 0;
    if (split) {
      // Split data across the two buffers
      remaining_space = LOG_BUFFER_SIZE - cur_buffer->idx;
      memcpy(cur_buffer->buffer + cur_buffer->idx, data, remaining_space);
      cur_buffer->idx = LOG_BUFFER_SIZE;
    }

    // Switch to the other buffer
    cur_buffer->full = true;

    *buffer_num = !(*buffer_num);
    cur_buffer = &double_buffer[*buffer_num];

    if (cur_buffer->idx != 0) {
      // If new buffer doesn't start at the beginning then something is wrong
      return DOUBLE_BUFFER_INDEX_ERROR;
    } else {
      // Write data to new buffer
      memcpy(cur_buffer->buffer, data + remaining_space,
             data_length - remaining_space);
      cur_buffer->idx += data_length;
    }
  } else {
    // If data fits in current buffer then write it
    memcpy(cur_buffer->buffer + cur_buffer->idx, data, data_length);
    cur_buffer->idx += data_length;
  }

  return DOUBLE_BUFFER_SUCCESS;
}

// ඞ
void control_function() {
  control_state = ControlFunctionState_init_default;
  control_state.cycle_start_us = micros();
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;

  // Grab sensor data
  noInterrupts();
  control_state.engine_count = engine_count;
  control_state.gear_count = gear_count;
  interrupts();

  // Calculate instantaneous RPMs
  control_state.engine_rpm = (control_state.engine_count - last_engine_count) /
                             ENGINE_COUNTS_PER_ROT / dt_s * SECONDS_PER_MINUTE;

  // TODO: Test gear RPM calculation
  float gear_rpm = GEAR_SAMPLE_WINDOW / GEAR_COUNTS_PER_ROT / gear_time_diff_us * US_PER_SECOND *
                   SECONDS_PER_MINUTE;

  last_engine_count = control_state.engine_count;

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  control_state.secondary_rpm = wheel_rpm * SECONDARY_TO_WHEEL_RATIO;

  // Controller
  control_state.target_rpm = ENGINE_TARGET_RPM;
  control_state.engine_rpm_error =
      control_state.engine_rpm - control_state.target_rpm;

  control_state.velocity_command = control_state.engine_rpm_error * ACTUATOR_KP;
  control_state.velocity_command =
      CLAMP(control_state.velocity_command, -ODRIVE_VELOCITY_LIMIT,
            ODRIVE_VELOCITY_LIMIT);

  actuator.set_velocity(control_state.velocity_command);

  // Populate control state
  control_state.inbound_limit_switch = actuator.get_inbound_limit();
  control_state.outbound_limit_switch = actuator.get_outbound_limit();
  control_state.engage_limit_switch = actuator.get_engage_limit();

  control_state.last_heartbeat_ms = odrive.get_time_since_heartbeat_ms();
  control_state.disarm_reason = odrive.get_disarm_reason();
  control_state.active_errors = odrive.get_active_errors();
  control_state.procedure_result = odrive.get_procedure_result();

  control_state.bus_current = odrive.get_bus_current();
  control_state.bus_voltage = odrive.get_bus_voltage();
  control_state.iq_measured = odrive.get_iq_measured();
  control_state.iq_setpoint = odrive.get_iq_setpoint();

  control_state.velocity_estimate = odrive.get_vel_estimate();
  control_state.position_estimate = odrive.get_pos_estimate();

  if (sd_initialized) {
    // Serialize control state
    size_t message_length = encode_pb_message(
        message_buffer, MESSAGE_BUFFER_SIZE, PROTO_CONTROL_FUNCTION_MESSAGE_ID,
        &ControlFunctionState_msg, &control_state);

    // Write to double buffer
    u8 write_status = write_to_double_buffer(
        message_buffer, message_length, double_buffer, &cur_buffer_num, false);
    if (write_status != 0) {
      Serial.printf("Error: Failed to write to double buffer with error %d\n",
                    write_status);
    }
  }
  control_cycle_count++;
}

void button_shift_mode() {
  noInterrupts();
  u32 cur_engine_count = engine_count;
  u32 cur_gear_count = gear_count;
  interrupts();

  bool button_pressed[5] = {false, false, false, false, false};
  for (size_t i = 0; i < 5; i++) {
    button_pressed[i] = !digitalRead(BUTTON_PINS[i]) && last_button_state[i];
  }
  for (size_t i = 0; i < 5; i++) {
    last_button_state[i] = digitalRead(BUTTON_PINS[i]);
  }

  Serial.printf("State: %d, Velocity: %f, In: %d, Out: %d, Engage: %d\n",
                odrive.get_axis_state(), odrive.get_vel_estimate(),
                actuator.get_inbound_limit(), actuator.get_outbound_limit(),
                actuator.get_engage_limit());

  float velocity = 10.0;
  if (button_pressed[0]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  } else if (button_pressed[1]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  } else if (button_pressed[2]) {
    actuator.set_velocity(-velocity);
  } else if (button_pressed[3]) {
    actuator.set_velocity(0.0);
  } else if (button_pressed[4]) {
    actuator.set_velocity(velocity);
  }

  control_cycle_count++;
}

void debug_mode() {
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;

  // Grab sensor data
  noInterrupts();
  control_state.engine_count = engine_count;
  control_state.gear_count = gear_count;
  interrupts();

  // Calculate instantaneous RPMs
  float engine_rpm = (control_state.engine_count - last_engine_count) /
                     ENGINE_COUNTS_PER_ROT / dt_s * SECONDS_PER_MINUTE;

  Serial.printf("Engine RPM: %f\n", engine_rpm);
  last_engine_count = control_state.engine_count;
}

void setup() {
  // Pin setup
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

  // Wait for serial if enabled
  if (wait_for_serial) {
    u32 led_flash_time_ms = 500;
    while (!Serial) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
    }
  }
  write_all_leds(LOW);

  // Setup RTC
  setSyncProvider(get_teensy3_time);

  bool rtc_set = timeStatus() == timeSet && year() > 2021;
  if (!rtc_set) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  // SD initialization
  sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
  if (!sd_initialized) {
    Serial.println("Warning: SD failed to initialize");
    digitalWrite(RED_LED_PIN, HIGH);
  } else {
    char log_name[64];
    u16 log_name_length = 0;

    if (rtc_set) {
      log_name_length = snprintf(
          log_name, sizeof(log_name), "log_%04d-%02d-%02d_%02d-%02d-%02d.bin",
          year(), month(), day(), hour(), minute(), second());
    } else {
      strncpy(log_name, "log_unknown_time.bin", sizeof(log_name));
      log_name_length = 20;
    }

    if (SD.exists(log_name)) {
      char log_name_duplicate[64];
      for (int log_num = 0; log_num < 1000; log_num++) {
        snprintf(log_name_duplicate, sizeof(log_name_duplicate),
                 "%.*s_%03d.bin", log_name_length - 4, log_name, log_num);
        if (!SD.exists(log_name_duplicate)) {
          break;
        }
      }
      strncpy(log_name, log_name_duplicate, sizeof(log_name));
      log_name[sizeof(log_name) - 1] = '\0';
    }
    Serial.printf("Info: Logging to %s\n", log_name);
    log_file = SD.open(log_name, FILE_WRITE);
  }

  // Attach sensor interrupts
  attachInterrupt(
      ENGINE_SENSOR_PIN, []() { ++engine_count; }, FALLING);

  attachInterrupt(
      GEARTOOTH_SENSOR_PIN,
      []() {
        u32 curr_time_us = micros();
        if(gear_count % GEAR_SAMPLE_WINDOW == 0) {
          gear_time_diff_us = curr_time_us - last_gear_time_us;
          last_gear_time_us = curr_time_us;
        }
        ++gear_count;
      },
      FALLING);

  // Attach limit switch interrupts
  attachInterrupt(
      LIMIT_SWITCH_IN_PIN, []() { odrive.set_input_vel(0, 0); }, FALLING);
  attachInterrupt(
      LIMIT_SWITCH_OUT_PIN,
      []() {
        odrive.set_input_vel(0, 0);
        odrive.set_absolute_position(0);
      },
      FALLING);

  // Initialize CAN bus
  flexcan_bus.begin();
  flexcan_bus.setBaudRate(FLEXCAN_BAUD_RATE);
  flexcan_bus.setMaxMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(can_parse);

  // Set interrupt priorities
  // TODO: Figure out proper ISR priority levels
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
  timer.priority(255);

  // Attach timer interrupt
  switch (operating_mode) {
  case OperatingMode::NORMAL:
    timer.begin(control_function, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::BUTTON_SHIFT:
    timer.begin(button_shift_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::DEBUG:
    timer.begin(debug_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::NONE:
    break;
  }
}

void loop() {
  // LED indicators
  digitalWrite(GREEN2_LED_PIN, actuator.get_outbound_limit());
  digitalWrite(WHITE_LED_PIN, actuator.get_inbound_limit());

  // Flush SD card if buffer full
  if (sd_initialized && log_file) {
    for (size_t buffer_num = 0; buffer_num < 2; buffer_num++) {
      if (double_buffer[buffer_num].full) {
        Serial.printf("Info: Writing buffer %d to SD\n", buffer_num);
        log_file.write(double_buffer[buffer_num].buffer,
                       double_buffer[buffer_num].idx);
        log_file.flush();
        double_buffer[buffer_num].full = false;
        double_buffer[buffer_num].idx = 0;
      }
    }
  }
}
