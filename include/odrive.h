#ifndef ODRIVE_H
#define ODRIVE_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <types.h>

// TODO: Move status codes
constexpr u8 ODRIVE_CMD_SUCCESS = 0;
constexpr u8 ODRIVE_CMD_ERROR_INVALID_AXIS = 1;
constexpr u8 ODRIVE_CMD_ERROR_INVALID_COMMAND = 2;
constexpr u8 ODRIVE_CMD_ERROR_WRITE_FAILED = 3;

// TODO: Move CAN IDs
constexpr u32 CAN_GET_VERSION = 0x000;
constexpr u32 CAN_HEARTBEAT = 0x001;
constexpr u32 CAN_ESTOP = 0x002;
constexpr u32 CAN_GET_ERRORS = 0x003;
constexpr u32 CAN_RXSDO = 0x004;
constexpr u32 CAN_TXSDO = 0x005;
constexpr u32 CAN_ADDRESS = 0x006;
constexpr u32 CAN_SET_AXIS_STATE = 0x007;
constexpr u32 CAN_GET_ENCODER_ESTIMATES = 0x009;
constexpr u32 CAN_SET_CONTROLLER_MODE = 0x00b;
constexpr u32 CAN_SET_INPUT_POS = 0x00c;
constexpr u32 CAN_SET_INPUT_VEL = 0x00d;
constexpr u32 CAN_SET_INPUT_TORQUE = 0x00e;
constexpr u32 CAN_SET_LIMITS = 0x00f;
constexpr u32 CAN_SET_TRAJ_VEL_LIMIT = 0x011;
constexpr u32 CAN_SET_TRAJ_ACCEL_LIMITS = 0x012;
constexpr u32 CAN_SET_TRAJ_INERTIA = 0x013;
constexpr u32 CAN_GET_IQ = 0x014;
constexpr u32 CAN_GET_TEMPERATURE = 0x015;
constexpr u32 CAN_REBOOT = 0x016;
constexpr u32 CAN_GET_BUS_VOLTAGE_CURRENT = 0x017;
constexpr u32 CAN_CLEAR_ERRORS = 0x018;
constexpr u32 CAN_SET_ABSOLUTE_POSITION = 0x019;
constexpr u32 CAN_SET_POS_GAIN = 0x01a;
constexpr u32 CAN_SET_VEL_GAINS = 0x01b;
constexpr u32 CAN_GET_TORQUES = 0x01c;
constexpr u32 CAN_GET_POWERS = 0x01d;
constexpr u32 CAN_ENTER_DFU_MODE = 0x01f;

/**
 * @brief This class provides methods to communicate with an ODrive over the CAN
 * bus.
 */
class ODrive {

public:
  enum class AxisState {
    UNDEFINED = 0x0,
    IDLE = 0x1,
    STARTUP_SEQUENCE = 0x2,
    FULL_CALIBRATION_SEQUENCE = 0x3,
    MOTOR_CALIBRATION = 0x4,
    ENCODER_INDEX_SEARCH = 0x6,
    ENCODER_OFFSET_CALIBRATION = 0x7,
    CLOSED_LOOP_CONTROL = 0x8,
    LOCKIN_SPIN = 0x9,
    ENCODER_DIR_FIND = 0xA,
    HOMING = 0xB,
    ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
    ENCODER_HALL_PHASE_CALIBRATION = 0xD,
    ANTICOGGING_CALIBRATION = 0xE,
  };

  ODrive(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus,
         u32 node_id = 0x3f)
      : flexcan_bus(flexcan_bus), node_id(node_id) {}

  u8 init();

  void parse_message(const CAN_message_t &msg);

  // Requesters
  u8 request_errors();
  u8 request_iq();
  u8 request_temperature();
  u8 request_bus_voltage_current();

  // Getters
  u32 get_time_since_heartbeat_ms();
  u32 get_axis_error();
  AxisState get_axis_state();
  u32 get_active_errors();
  u32 get_disarm_reason();
  float get_vel_estimate();
  float get_pos_estimate();
  float get_iq_setpoint();
  float get_iq_measured();
  float get_bus_voltage();
  float get_bus_current();

  // Commands
  u8 reboot();
  u8 clear_errors();

  // Setters
  u8 set_axis_state(AxisState axis_state);
  u8 set_controller_mode(u32 control_mode, u32 input_mode);
  u8 set_input_pos(float input_pos, i16 vel_ff, i16 torque_ff);
  u8 set_input_vel(float input_vel, float torque_ff);
  u8 set_input_torque(float input_torque);
  u8 set_limits(float current_limit, float vel_limit);
  u8 set_traj_vel_limit(float traj_vel_limit);
  u8 set_traj_accel_limits(float traj_accel_limit, float traj_decel_limit);
  u8 set_traj_intertia(float traj_inertia);
  u8 set_absolute_position(float pos_estimate);
  u8 set_pos_gain(float pos_gain);
  u8 set_vel_gains(float vel_gain, float vel_integrator_gain);

private:
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus;

  u32 node_id;

  u32 last_heartbeat_ms;

  u32 axis_error;
  AxisState axis_state;
  u8 procedure_result;
  u8 trajectory_done_flag;
  u32 active_errors, disarm_reason;
  float vel_estimate, pos_estimate;
  float iq_setpoint, iq_measured;
  float bus_voltage, bus_current;

  u8 send_command(u32 cmd_id, bool remote, u8 buf[8]);
  u8 send_empty_command(u32 cmd_id, bool remote);
};
#endif
