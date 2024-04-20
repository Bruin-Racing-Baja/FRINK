#include <actuator.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))

/**
 * Constructor for the actuator
 * @param odrive Pointer to ODrive object
 */
Actuator::Actuator(ODrive *odrive, bool do_ramping = false) : odrive(odrive), trapezoid_mode(do_ramping) {}

/**
 * Initializes connection to physical ODrive
 * @return 0 if successful
 */
u8 Actuator::init() { return 0; }

/**
 * Instructs ODrive to attempt encoder homing
 * @return 0 if successful
 */
u8 Actuator::home_encoder() {
  // TODO: Add timeout
  if (odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL) != 0) {
    return HOME_CAN_ERROR;
  }

  // Move in to engaged limit if we start on outbound limit
  if (get_outbound_limit()) {
    while (!get_engage_limit()) {
      // TODO: Why does this have to be set in the loop?
      set_velocity(ACTUATOR_HOME_VELOCITY);
      delay(100);
    }
    set_velocity(0);
  }

  // Move out to outbound limit
  set_velocity(-ACTUATOR_HOME_VELOCITY);
  while (!get_outbound_limit()) {
    delay(100);
  }
  set_velocity(0);

  odrive->set_absolute_position(0);

  return HOME_SUCCCESS;
}

/** Instructs the ODrive object to set given velocity
 * @param velocity The velocity to set
 * @return 0 if successful
 */
u8 Actuator::set_velocity(float velocity) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
                                  ODrive::INPUT_MODE_VEL_RAMP) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  if (get_inbound_limit() && velocity > 0) {
    odrive->set_input_vel(0, 0);
    return SET_VELOCITY_IN_LIMIT_SWITCH_ERROR;
  }

  if (get_outbound_limit() && velocity < 0) {
    odrive->set_input_vel(0, 0);
    return SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR;
  }

  //trapezoid velocity limit
  float lower_vel_limit = -ODRIVE_VEL_LIMIT;  
  float upper_vel_limit = ODRIVE_VEL_LIMIT;
  if (trapezoid_mode) {  
      float temp_pos = odrive->get_pos_estimate();
      //verify ramp positions do not overlap
      if (ACTUATOR_ENGAGE_RAMP_POS_ROT < ACTUATOR_INBOUND_RAMP_POS_ROT) {
        if (temp_pos < ACTUATOR_ENGAGE_RAMP_POS_ROT) {
          //lower limit sets to negative linear function, upper limit left alone
          lower_vel_limit = 
            (-ODRIVE_VEL_LIMIT / (ACTUATOR_ENGAGE_RAMP_POS_ROT - ACTUATOR_ENGAGE_POS_ROT)) * (temp_pos - ACTUATOR_ENGAGE_RAMP_POS_ROT) - ODRIVE_VEL_LIMIT;
          upper_vel_limit = ODRIVE_VEL_LIMIT;
        }
        else if (temp_pos < ACTUATOR_INBOUND_RAMP_POS_ROT) {
          //just a rectangle
          lower_vel_limit = -ODRIVE_VEL_LIMIT;
          upper_vel_limit = ODRIVE_VEL_LIMIT;
        }
        else {
          //lower limit left alone, upper limit sets to positive linear function
          lower_vel_limit = -ODRIVE_VEL_LIMIT;
          upper_vel_limit = 
            (ODRIVE_VEL_LIMIT / (ACTUATOR_INBOUND_RAMP_POS_ROT - ACTUATOR_INBOUND_POS_ROT)) * (temp_pos - ACTUATOR_INBOUND_RAMP_POS_ROT) + ODRIVE_VEL_LIMIT;
        }
      }
  }
  velocity = CLAMP(velocity, lower_vel_limit, upper_vel_limit);


  if (odrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity_mode = true;

  return SET_VELOCITY_SUCCCESS;
}

/** Instructs the ODrive object to set given position
 * @param position The position to set
 * @return 0 if successful
 */
u8 Actuator::set_position(float position) {
  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_POSITION_CONTROL,
                                  ODrive::INPUT_MODE_TRAP_TRAJ) != 0) {
    return SET_POSITION_CAN_ERROR;
  }

  if (position < 0.0 || ACTUATOR_INBOUND_POS_ROT < position) {
    return SET_POSITION_LIMIT_SWITCH_ERROR;
  }

  if (odrive->set_input_pos(position, 0, 0) != 0) {
    return SET_POSITION_CAN_ERROR;
  }

  velocity_mode = false;

  return SET_POSITION_SUCCCESS;
}

bool Actuator::get_inbound_limit() { return !digitalRead(LIMIT_SWITCH_IN_PIN); }

bool Actuator::get_outbound_limit() {
  return !digitalRead(LIMIT_SWITCH_OUT_PIN);
}

bool Actuator::get_engage_limit() {
  return !digitalRead(LIMIT_SWITCH_ENGAGE_PIN);
}
