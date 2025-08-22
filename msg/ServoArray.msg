# ServoArray.msg
# Author: Treggon Owens
# Description: Custom message for controlling 16 servos and reading 2 PWM inputs

uint8[16] angles
bool[16] enabled
uint16 pwm_input_1
uint16 pwm_input_2
