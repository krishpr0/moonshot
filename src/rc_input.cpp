#include "rc_input.h"
#include "crsf.h"

HardwareSerial CRSF_UART(PA3, PA2); //RX, TX

void rc_init() {
    crsf_init(&CRSF_UART);
}

bool rc_read_channels(uint16_t* channels, uint16_t* num_channels) {
    return crsf_parse_packet(channels, num_channels);
    
}