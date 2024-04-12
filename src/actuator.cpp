#include <actuator.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

/**
 * Constructor for the actuator
 * @param odrive Pointer to ODrive object
 */
Actuator::Actuator(ODrive *odrive) : odrive(odrive) {}

/**
 * Initializes connection to physical ODrive
 * @return 0 if successful
 */
u8 Actuator::init() { return 0; }

/**
 * Instructs ODrive to attempt encoder homing
 * @return 0 if successful
 */
u8 Actuator::encoder_index_search() {
  // TODO: Fix delay
  if (odrive->set_axis_state(ODrive::AXIS_STATE_ENCODER_INDEX_SEARCH) != 0) {
    return INDEX_SEARCH_CAN_ERROR;
  }
  delayMicroseconds(5e6);
  return INDEX_SEARCH_SUCCCESS;
}

/** Instructs the ODrive object to set given velocity
 * @param velocity The velocity to set
 * @return 0 if successful
 */
u8 Actuator::set_velocity(float velocity) {
  if (get_inbound_limit() && velocity > 0) {
    odrive->set_input_vel(0, 0);
    return SET_VELOCITY_IN_LIMIT_SWITCH_ERROR;
  }
  if (get_outbound_limit() && velocity < 0) {
    odrive->set_input_vel(0, 0);
    return SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR;
  }

  if (odrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  return SET_VELOCITY_SUCCCESS;
}

bool Actuator::get_inbound_limit() { return !digitalRead(LIMIT_SWITCH_IN_PIN); }

bool Actuator::get_outbound_limit() {
  return !digitalRead(LIMIT_SWITCH_OUT_PIN);
}

bool Actuator::get_engage_limit() {
  return !digitalRead(LIMIT_SWITCH_ENGAGE_PIN);
}
