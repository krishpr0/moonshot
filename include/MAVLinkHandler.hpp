#pragma once
#include <mavlink/common/mavlink>

class MAVLinkHandler {
    private:
        mavlink_system_t sys_ = {1, 1};

        public : 
        void sendHeartbeat() {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_heartbeat_pack(sys_.sysid, sys_.compid, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
        }

        void sendAttitude(float roll, float pitch, float yaw) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_attitude_pack(sys_.sysid, sys_.compid, &msg, millis(), roll, pitch, yaw, 0, 0, 0);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
        }

        void sendParam(const char* name, float value) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_param_value_pack(sys_.sysid, sys_.compid, &msg, name, value, MAV_PARAM_TYPE_REAL32, 10, 0);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
        }
};