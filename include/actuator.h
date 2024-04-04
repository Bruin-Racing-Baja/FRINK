#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Actuator {
public:
  Actuator(ODrive *odrive);

  u8 init();
  u8 encoder_index_search();
  u8 set_velocity(float velocity_offset);

private:
  ODrive *odrive;
};

#endif
