#pragma once

#include <stdint.h>
#include <../defines.h>

#define CRSF_FRAME_SIZE_MAX 30

#define CRSF_PAYLOAD_SIZE_MAX (CRSF_FRAME_SIZE_MAX-4)


struct crsf_frame_header_t {
    uint8_t device_address;
    uint8_t length;
};

struct crsf_frame_t {
    crsf_frame_header_t header;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];
};


#define RC_INPUT_MAX_CHANNELS 18


int crsf_config(int uart_fd);

bool crsf_parse(const uint8_t *frame, unsigned len, uint16_t *values, uint16_t *num_values, uint16_t max_channels);

bool crsf_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining);

bool crsf_send_telemetry_gps(int uart_fd, uint32_t latitude, int32_t longtitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);

bool crsf_send_telemetry_attitude(int uart_fd,  int16_t pitch, int16_t roll, int16_t yaw);

bool crsf_send_telemetry_flight_mode(int uart_fd, const char *flight_mode);