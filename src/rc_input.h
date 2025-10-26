#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>

#define RC_MAX_CHANNELS 16

void rc_init();
bool rc_read_channels(uint16_t* channels, uint16_t* num_channels);

#endif