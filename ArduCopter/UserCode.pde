/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "include/mavlink/v1.0/ardupilotmega/mavlink.h"

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
  static mavlink_message_t msg;
  static mavlink_status_t status;
  static uint8_t buf[128];

#define SEND_ACK(type) do {\
  mavlink_msg_mission_ack_pack(0, 0, &msg, 0, 0, type); \
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); \
  hal.uartC->write(buf, len);\
} while(0)

#define SEND_MISSION_ITEM() do {\
  mavlink_msg_mission_item_pack(0, 0, &msg, 0, 0, 0, 0, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home.lat, home.lng, 0); \
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); \
  hal.uartC->write(buf, len);\
} while(0)


  while (hal.uartC->available() > 0) {
    int16_t c = hal.uartC->read();
    if (c < 0) {
      break;
    }
    if (mavlink_parse_char(MAVLINK_COMM_2, (uint8_t)c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_ITEM:
        {
          mavlink_mission_item_t packet;
          mavlink_msg_mission_item_decode(&msg, &packet);

          switch (packet.command) {
            case MAV_CMD_NAV_WAYPOINT:
              Vector3f target_pos = pv_latlon_to_vector(
                  1.0e7f * packet.x, 1.0e7f * packet.y, 1.0e2f * packet.z);

              // sanity check
              if (target_pos.x < -2000.0f || target_pos.x > 2000.0f
                  || target_pos.y < -2000.0f || target_pos.y > 2000.0f
                  || target_pos.z < 0 || target_pos.z > 3500.0f) {
                // invalid position
                SEND_ACK(MAV_MISSION_ERROR);
                SEND_MISSION_ITEM();
                break;
              }

              // send ack
              SEND_ACK(MAV_MISSION_ACCEPTED);

              if (control_mode != GUIDED) {
                set_mode(GUIDED);
              }

              wp_nav.set_destination(target_pos);

              // Update loiter target position
              //wp_nav.set_loiter_target(target_pos);

              break;
          }
        }
      }
    }
  }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
