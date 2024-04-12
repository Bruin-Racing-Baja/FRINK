#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Actuator {

public:
  static const u8 SET_VELOCITY_SUCCCESS = 0;
  static const u8 SET_VELOCITY_IN_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_CAN_ERROR = 2;

  static const u8 INDEX_SEARCH_SUCCCESS = 0;
  static const u8 INDEX_SEARCH_CAN_ERROR = 1;
  static const u8 INDEX_SEARCH_FAILED = 2;

  Actuator(ODrive *odrive);

  u8 init();
  u8 encoder_index_search();
  u8 set_velocity(float velocity_offset);
  bool get_inbound_limit();
  bool get_outbound_limit();
  bool get_engage_limit();

private:
  ODrive *odrive;
};

#endif
