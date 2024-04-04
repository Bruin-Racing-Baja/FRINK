#include <actuator.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

/**
 * Constructor for the actuator
 * @param odrive Pointer to ODrive object
 */
Actuator::Actuator(ODrive *odrive)
    : odrive(odrive)  {}

/**
 * Initializes connection to physical ODrive
 * @return true if successful
 */
u8 Actuator::init() { return 0; }

/**
 * Instructs ODrive to attempt encoder homing
 * @return 0 if successful
 */
u8 Actuator::encoder_index_search() {
  // TODO: Fix delay
  if(odrive->set_axis_state(ODrive::AxisState::ENCODER_INDEX_SEARCH) != 0){
    return 1;
  }
  delayMicroseconds(5e6);
  return 0;
}

/** Instructs the ODrive object to set given velocity
 * @param velocity The velocity to set
 * @return 0 if successful
 */
u8 Actuator::set_velocity(float velocity, float brake_offset) {
  // TODO: Fix error handling. Implement brake offset.
  if (odrive->set_axis_state(ODrive::AxisState::CLOSED_LOOP_CONTROL) != 0) {
    Serial.print("Error: Could not set ODrive to CLOSED_LOOP_CONTROL state\n");
    return 1;
  }

  if (odrive->set_input_vel(velocity, 0) != 0) {
    Serial.print("Error: Could not set ODrive velocity\n");
    return 1;
  }

  return 0;
}
