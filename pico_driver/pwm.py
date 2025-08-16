from machine import Pin, PWM
import hardware

#@duty should be a number between 0 and 1 representing the duty cycle
#@pwm the pwm object for the claw specifically
def set_pwm_duty(duty, gen_pwm: PWM):
    if ((duty < 0) or (duty > 65535)):
        return -1
    
    #in micropython the duty cycle is calculated by duty_u16 / 65535
    gen_pwm.duty_u16(duty) 

#Apparently the claw motor doesn't like sudden voltages, so this builds the duty cycle up from 0 to 1
def ramp_claw_speed(claw_pwm: PWM):
    maxDuty = 257
    for i in range(maxDuty):
        set_pwm_duty(i * 255, claw_pwm)

def stop_claw(claw_pwm: PWM):
    set_pwm_duty(0, claw_pwm)


def solenoid_on(solenoid_pwm: PWM):
    solenoid_pwm.duty_u16(65535)

def solenoid_off(solenoid_pwm: PWM):
    solenoid_pwm.duty_u16(0)

