import time
from machine import I2C

PWM_EXP_ADDR = 0x40
INTERNAL_FREQ = 25000000

def pwm_exp_write_reg(i2c: I2C, reg: bytearray, val: bytearray):
    data = bytearray(2)
    data[0] = reg[0]
    data[1] = val[0]

    i2c.writeto(PWM_EXP_ADDR, data)

def pwm_exp_read_reg(i2c: I2C, reg: bytearray):
    data = bytearray(1)
    data[0] = reg[0]

    i2c.writeto(PWM_EXP_ADDR, data)
    return i2c.readfrom(PWM_EXP_ADDR, 1)

def pwm_exp_set_freq(i2c: I2C, freq: int):
    prescaleVal = (INTERNAL_FREQ / (4096 * freq)) - 1

    if(prescaleVal > 255):
        prescaleVal = 255
    
    if(prescaleVal < 3):
        prescaleVal = 3

    mode1data = bytearray(1)
    mode1data = pwm_exp_read_reg(i2c, 0x00)

    #sleep mode
    pwm_exp_write_reg(i2c, 0x00, ((mode1data[0] &~ 0x80) | 0x10))
    #write freq
    pwm_exp_write_reg(0xFE, prescaleVal)

    #start up again
    pwm_exp_write_reg(0x00, ((mode1data[0] & ~0x10) | 0x80))

    time.sleep_us(500)

def pwm_exp_set_duty(i2c: I2C, channel: int, duty):
    data = bytearray(5)
    
