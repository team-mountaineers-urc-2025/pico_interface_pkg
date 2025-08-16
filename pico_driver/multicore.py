import time, _thread, machine
from machine import I2C, PWM
from collections import deque
import pca
import pwm


#h1 end effector h2 solenoid

#This will continuously check for messages from the communications thread, and if there is one take action
#get a command, do thing for a bit, if no message after a certain amount of time, stop
def core_1_task(q: deque, coreLock: _thread.LockType, claw_pwm: PWM, i2c: I2C ):
    while True:
        if (coreLock.acquire(blocking=False)):
            try:
                msg = q.pop()
            except:
                pwm.stop_claw(claw_pwm)
                continue

            coreLock.release()
        
            #open the claw
            if(msg == '5'):
                pca.set_claw_dir_open(i2c)
                pwm.ramp_claw_speed(claw_pwm)
                pwm.stop_claw(claw_pwm)

            #stop the claw, shouldn't need to be done most of the time
            if(msg == '6'):
                pwm.stop_claw(claw_pwm)

            #close the claw
            elif(msg == '7'):
                pca.set_claw_dir_close(i2c)
                pwm.ramp_claw_speed(claw_pwm)
                pwm.stop_claw(claw_pwm)
            
            #stop everything if message invalid somehow
            else:
                pwm.stop_claw(claw_pwm)