#ifndef DASH_H
#define DASH_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <types.h>
#include <constants.h>

/**
 * @brief This class provides methods to communicate with an ODrive over the CAN
 * bus.
 */
class Dash {



public:
//SENDING FROM FRINK TO DASH TEENSY
  static const u32 EG_RPM = 0x000;
  static const u32 WHEEL_RPM = 0x001;
  static const u32 ACTUATOR_POS = 0x002;
  static const u32 TARGET_RPM = 0x00f;

//SENDING AND RECIEVING
  static const u32 LOW_SP_TARG_RPM = 0x003;
  static const u32 HI_SP_TARG_RPM = 0x004;
  static const u32 P_GAIN = 0x005;
  static const u32 D_GAIN = 0x006;
  
//RECIEVING FROM DASH TEENSY TO FRINK
  static const u32 BUTTON1 = 0x007;
  static const u32 BUTTON2 = 0x009;
  static const u32 BUTTON3 = 0x00b;
  static const u32 BUTTON4 = 0x00c;
  static const u32 BUTTON5 = 0x00d;
  static const u32 CLUTCH = 0x00e;

//ERRORS
  static const u8 CMD_SUCCESS = 0;
  static const u8 CMD_ERROR_INVALID_AXIS = 1;
  static const u8 CMD_ERROR_INVALID_COMMAND = 2;
  static const u8 CMD_ERROR_WRITE_FAILED = 3;

  Dash(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus, u32 node_id, MutableConstants *constants);

  void parse_message(const CAN_message_t &msg);
  u8 clutch_flag = false;
//Setters
u8 set_engine_rpm(float engine_rpm);
u8 set_wheel_rpm(float wheel_rpm);
u8 set_actuator_pos(float actuator_pos);
u8 set_targ_rpm(float targ_rpm);
u8 set_lo_spd_targ_rpm(float lo_targ_rpm);
u8 set_hi_spd_targ_rpm(float hi_targ_rpm);
u8 set_p_gain(float p_gain);
u8 set_d_gain(float d_gain);


private:
  MutableConstants* constants;
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus;

  u32 node_id;

  u8 send_command(u32 cmd_id, bool remote, u8 buf[8]);
};

#endif //DASH_H