#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <math.h>
#include <stddef.h>
#include <types.h>

#define dancing 13

// Units
constexpr float SECONDS_PER_MINUTE = 60.0; // s / min
constexpr float MS_PER_SECOND = 1.0e3;     // ms / s
constexpr float US_PER_SECOND = 1.0e6;     // us / s
constexpr float SECONDS_PER_MS = 1.0e-3;   // s / ms
constexpr float SECONDS_PER_US = 1.0e-6;   // s / us

constexpr float MM_PER_INCH = 25.4;              // mm / inch
constexpr float INCHES_PER_MM = 1 / MM_PER_INCH; // inch / mm

constexpr float FEET_PER_MILE = 5280.0; // feet / mile
constexpr float INCH_PER_FEET = 12.0;   // inch / feet

// Powertrain
constexpr float ENGINE_COUNTS_PER_ROT = 16; // count / rot
constexpr float GEAR_COUNTS_PER_ROT = 6;    // count / rot

// NOTE: x_to_y ratio is how many rotations of x for 1 rotation of y
constexpr float GEAR_TO_WHEEL_RATIO = 1.0;
constexpr float SECONDARY_TO_WHEEL_RATIO =
    ((46.0 / 17.0) * (56.0 / 19.0)); // ~7.975
constexpr float WHEEL_TO_SECONDARY_RATIO =
    (1.0 / SECONDARY_TO_WHEEL_RATIO); // ~0.1253
//
constexpr float WHEEL_DIAMETER_INCH = 23.0; // inch
constexpr float WHEEL_MPH_PER_RPM =
    (WHEEL_DIAMETER_INCH * M_PI) / (FEET_PER_MILE * INCH_PER_FEET); // mph / rpm

// Actuator
constexpr float ACTUATOR_PITCH_MM = 5.0; // mm / rot
constexpr float ACTUATOR_PITCH_INCH =
    ACTUATOR_PITCH_MM * INCHES_PER_MM;            // inch / rot
constexpr float ACTUATOR_OUTBOUND_POS_INCH = 3.5; // inch
constexpr float ACTUATOR_BELT_POS_INCH = 1.0;       // inch

// Control Function
constexpr u32 CONTROL_FUNCTION_INTERVAL_MS = 10; // ms
constexpr float ENGINE_TARGET_RPM = 300.0; // rpm
constexpr float ACTUATOR_KP = 0.01;

// Teensy Pins
constexpr u8 GREEN_LED_PIN = 7;
constexpr u8 YELLOW_LED_PIN = 8;
constexpr u8 RED_LED_PIN = 9;
constexpr u8 WHITE_LED_PIN = 29;
constexpr u8 GREEN2_LED_PIN = 28;

constexpr u8 BUTTON_PINS[] = {2, 3, 4, 5, 6};

constexpr u8 ENGINE_SENSOR_PIN = 15;
constexpr u8 GEARTOOTH_SENSOR_PIN = 14;

constexpr u8 THROTTLE_POT_PIN = 21;
constexpr u8 BRAKE_SENSOR_PIN = 20;

// Flexcan
constexpr u32 FLEXCAN_BAUD_RATE = 250000;
constexpr u32 FLEXCAN_MAX_MAILBOX = 16;

// Logging
// bytes_per_cycle * cycle_freq * time_to_flush_sd * safety_factor
// 100 * 100 * 0.4 * 2 = 8000
constexpr size_t LOG_BUFFER_SIZE = 8192; // 12288 to be safe
constexpr size_t CONTROL_FUNCTION_MESSAGE_BUFFER_SIZE = 512;

constexpr u8 PROTO_HEADER_MESSAGE_ID = 0x00;
constexpr u8 PROTO_CONTROL_FUNCTION_MESSAGE_ID = 0x01;

// ODrive Pins
// TODO: Get ODrive limit switch pins
const u8 LIMIT_SWITCH_IN_ODRIVE_PIN = 7;
const u8 LIMIT_SWITCH_OUT_ODRIVE_PIN = 8;

#endif
