#include <FlexCAN_T4.h>
#include <dash.h>

Dash::Dash(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus, u32 node_id):flexcan_bus(flexcan_bus), node_id(node_id)
{}

u8 Dash::send_command(u32 cmd_id, bool remote, u8 buf[8]) {
  // TODO: Fix error messages
  CAN_message_t msg;

  if (cmd_id < 0x000 || 0x00f < cmd_id) {
      Serial.println("invalid command");
    return Dash::CMD_ERROR_INVALID_COMMAND;
  }

  msg.id = (node_id << 5) | cmd_id;
  msg.len = 8;
  memcpy(&msg.buf, buf, 8);
  msg.flags.remote = remote;

  int write_code = flexcan_bus->write(msg);
  if (write_code == -1) {
    return Dash::CMD_ERROR_WRITE_FAILED;
  }
  return Dash::CMD_SUCCESS;
}

void Dash::parse_message(const CAN_message_t &msg)
{
     u32 parsed_node_id = (msg.id >> 5) & 0x3F;

  if (parsed_node_id != node_id) {
    return;
  }

  u32 cmd_id = msg.id & 0x1F;

  switch (cmd_id) {
  case BUTTON1:
  //Does something
    break;
  case BUTTON2:
    //Does something
    break;
  case BUTTON3:
    //Does something
    break;
  case BUTTON4:
    //Does something
    break;
  case BUTTON5:
    //Does something
    break;
  case P_GAIN:
    // set the controllers p gain to whatever was in the can message
    // memcpy(&ACTUATOR_KP, msg.buf,4);
    break;
  case D_GAIN:
    // set the controllers d gain to whatever was in the can message
    // memcpy(&ACTUATOR_KD, msg.buf,4);

    break;
  case LOW_SP_TARG_RPM:
    //set controllers low speeed target RPM to what was in the can message
    break;
  case HI_SP_TARG_RPM:
    //set controllers low speeed target RPM to what was in the can message
    memcpy(&ENGINE_TARGET_RPM, msg.buf,4);
    break; 
  case CLUTCH:
    //memcpy(&CLUTCH_FLAG, msg.buf,1);
    break;
  }


  
}

u8 Dash::set_engine_rpm(float engine_rpm)
{
    u8 buf[8] = {0};
  memcpy(buf, &engine_rpm, 4);
  return send_command(EG_RPM, false, buf);
}
u8 Dash::set_wheel_rpm(float wheel_rpm)
{
    u8 buf[8] = {0};
  memcpy(buf, &wheel_rpm, 4);
  return send_command(WHEEL_RPM, false, buf);
}
u8 Dash::set_actuator_pos(float actuator_pos)
{
    u8 buf[8] = {0};
  memcpy(buf, &actuator_pos, 4);
  return send_command(ACTUATOR_POS, false, buf);
}
u8 Dash::set_targ_rpm(float targ_rpm){
    u8 buf[8] = {0};
  memcpy(buf, &targ_rpm, 4);
  return send_command(TARGET_RPM, false, buf);
}

u8 Dash::set_lo_spd_targ_rpm(float lo_targ_rpm)
{
    u8 buf[8] = {0};
  memcpy(buf, &lo_targ_rpm, 4);
  return send_command(LOW_SP_TARG_RPM, false, buf);
}
u8 Dash::set_hi_spd_targ_rpm(float hi_targ_rpm)
{
    u8 buf[8] = {0};
  memcpy(buf, &hi_targ_rpm, 4);
  return send_command(HI_SP_TARG_RPM, false, buf);
}
u8 Dash::set_p_gain(float p_gain)
{
    u8 buf[8] = {0};
  memcpy(buf, &p_gain, 4);
  return send_command(P_GAIN, false, buf);
}
u8 Dash::set_d_gain(float d_gain)
{
    u8 buf[8] = {0};
  memcpy(buf, &d_gain, 4);
  return send_command(D_GAIN, false, buf);
}
