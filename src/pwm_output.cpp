#include "pwm_output.h"

#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 8


const uint8_t motor_pins[4] = {PA0, PA1, PA4, PA5};

void pwm_init() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(motor_pins[i], OUTPUT);
        analogWriteFrequency(PWM_FREQUENCY);
        analogWrite(motor_pins[i], 0);
    }
}

void pwm_output(float m1, float m2, float m3, float m4) {
    uint8_t duty1 = map(m1, 0, 100, 26, 51);
    uint8_t duty2 = map(m2, 0, 100, 26, 51);
    uint8_t duty3 = map(m3, 0, 100, 26, 51);
    uint8_t duty4 = map(m4, 0, 100, 26, 51);


    analogWrite(motor_pins[0], duty1);
    analogWrite(motor_pins[1], duty2);
    analogWrite(motor_pins[2], duty3);
    analogWrite(motor_pins[3], duty4);
}
