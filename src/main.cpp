#include <Arduino.h>
#include "rc_input.h"
#include "sensor.h"
#include "pid.h"
#include "motor_mix.h"
#include "pwm_output.h"

#define LED_PIN PC13 
#define PID_UPDATE_INTERVAL 5

uint16_t rc_channels[RC_MAX_CHANNELS];