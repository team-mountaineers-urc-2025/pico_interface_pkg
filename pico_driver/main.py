import select, sys, time, machine, gc
from collections import deque
import onewire
import ds18x20
import _thread
import pca
import multicore
import hardware
import pwm

#125 MHz clock freq
machine.freq(125000000)

# Set up the poll object
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

##
#queue for communication between cores, max size is kind of arbitrary at this point
#for now the messages are as follows:
# '5' means open the claw
# '6' is an explicit stop command, but shouldn't be necessary
# '7' means close the claw

# '1' means activate the solenoid
# '0' means deactivate the solenoid, with no power it automatically retracts, so no reversal necessary
#q = deque([], 15)
#validMsgs = ['5', '6', '7', '1', '0']



led = machine.Pin(25, machine.Pin.OUT)
#everything needs i2c
i2c = hardware.i2c_init()
devs = i2c.scan()
sys.stdout.write("I2C devices:" + str(devs) + '\n')
led.value(1)

pca.pca_init_all_out(pca.pca0, i2c)

mode_control = hardware.mode_control_init()

wdt = machine.WDT(timeout=6000)


#If the input on the mode control pin is high, enter science mode
if(mode_control.value()):
    sys.stdout.write("Science!")
    activeMaybe = False
    
    #science stuff
    soil_collector1_pwm = hardware.soil_collector1_lin_act_init()
    soil_collector2_pwm = hardware.soil_collector2_lin_act_init()

    probe_pwm_signal = hardware.probe_signal_init()
    drill_pwm = hardware.drill_lin_act_init()

    halogen_pin = hardware.halogen_bulb_init()
    halogen_pin.value(0)
    hb_extender = 0
    hb_active = False

    u2d2_pin = hardware.u2d2_relay_init()

    moisture_adc = hardware.moisture_sens_ADC_init()

    temp_ow = hardware.temp_ow_init()
    temp_sens = hardware.temp_init(temp_ow)


    pwm.set_pwm_duty(3300, soil_collector1_pwm)
    pwm.set_pwm_duty(3300, soil_collector2_pwm)
    pwm.set_pwm_duty(3300, probe_pwm_signal)
    pwm.set_pwm_duty(3630, drill_pwm)


    
    def timee(a):
        global activeMaybe
        if(activeMaybe == False):
            led.value(0)
        else:
            activeMaybe = False

    def temp_moist(a):
        try:
            moisture_val = str(moisture_adc.read_u16())
            #After calling convert temp you need to give time for the sensor to actually convert its data
            temp_sens.convert_temp()
            temp_dev_id = temp_sens.scan()[0]
            time.sleep_ms(750)
            temperature = temp_sens.read_temp(temp_dev_id)
            sys.stdout.write(f"{moisture_val},{temperature}\n")
        except:
            return

    def halogen_bulb_safety(a):
        global hb_active
        global hb_extender
        if(hb_extender != 3):
            hb_extender = hb_extender + 1
            return
        else:
            halogen_pin.value(0)
            hb_active = False            

    def input_flush(a):
        global poll_obj
        poll_result = poll_obj.poll(0)
        while (poll_result != []):
            poll_result = poll_obj.poll(0)
            dump = sys.stdin.readline()
        return

            
    emergencyTim = machine.Timer(period = 100, mode = machine.Timer.PERIODIC, callback = timee)
    temp_moist_data_timer = machine.Timer(period = 4000, mode = machine.Timer.PERIODIC, callback = temp_moist)
    hb_safety_timer = machine.Timer(period = 6500, mode = machine.Timer.PERIODIC, callback = halogen_bulb_safety)
    input_flush_timer = machine.Timer(period = 5000, mode = machine.Timer.PERIODIC, callback = input_flush)
    
    while True:
        poll_result = poll_obj.poll(5)
        wdt.feed()
        if (poll_result == []):
            continue
        data = sys.stdin.readline().strip("\n")

        #science controls
        if(data == "sc1"):
            led.value(1)
            data2 = sys.stdin.readline().strip("\n")
            try:
                duty = float(data2)
                pwm.set_pwm_duty((int(duty * 3300) + 3300), soil_collector1_pwm)
            except:
                continue

        elif(data == "sc2"):
            led.value(1)
            data2 = sys.stdin.readline().strip("\n")
            try:
                duty = float(data2)
                pwm.set_pwm_duty((int(duty * 3300) + 3300), soil_collector2_pwm)
            except:
                continue

        elif(data == "probe"):
            led.value(1)
            data2 = sys.stdin.readline().strip("\n")
            try:
                duty = float(data2)
                pwm.set_pwm_duty((int(duty * 3300) + 3300), probe_pwm_signal)
            except:
                continue

        #drill goes from 1.1 ms to 1.9 ms for retracted and extended instead of 1 ms and 2 ms like the other actuators
        elif(data == "drill"):
            led.value(1)
            data2 = sys.stdin.readline().strip("\n")
            try:
                duty = float(data2)
                pwm.set_pwm_duty((int(duty * 2583) + 3630), drill_pwm)
            except:
                continue


        elif(data == "moist"):
            led.value(1)
            moisture_val = str(moisture_adc.read_u16())
            sys.stdout.write(f"{moisture_val}\n")

        elif(data == "temp"):
            led.value(1)
            #After calling convert temp you need to give time for the sensor to actually convert its data
            temp_sens.convert_temp()
            temp_dev_id = temp_sens.scan()
            time.sleep_ms(750)
            temperature = temp_sens.read_temp(temp_dev_id)
            sys.stdout.write(f"{temperature}\n")


        #halogen bulb control
        elif(data == "hbon"):
            if(hb_active == True):
                continue
            hb_active = True
            halogen_pin.value(1)
            hb_safety_timer = machine.Timer(period = 6500, mode = machine.Timer.ONE_SHOT, callback = halogen_bulb_safety)
    
        elif(data== "hboff"):
            hb_active = False
            halogen_pin.value(0)

        #u2d2 relay stuff goes here
        elif(data == "u2d2on"):
            led.value(1)
            u2d2_pin.value(0)
    
        elif(data== "u2d2off"):
            led.value(0)
            u2d2_pin.value(1)


        elif(data=='reset'):
            machine.reset()

        else:
            led.value(0)
    

else:
    sys.stdout.write("Manipulation!")
    activeFlagClaw = False
    activeFlagSolenoid = False
    
    #arm stuff
    claw_pwm = hardware.claw_init()
    solenoid_pin = hardware.solenoid_init()
    #adc0, adc1 = hardware.fsr_init()
    
    pwm.stop_claw(claw_pwm)
    solenoid_pin.value(0)
    
    def timee(a):
        global activeFlagClaw
        global activeFlagSolenoid
        if(activeFlagClaw == False):
            led.value(0)
            pwm.stop_claw(claw_pwm)
        else:
            activeFlagClaw = False
        if(activeFlagSolenoid == False):
            led.value(0)
            solenoid_pin.value(0)
        else:
            activeFlagSolenoid = False
        
        
    def send_FSR(a):
        adcstr0 = str(adc0.read_u16())
        adcstr1 = str(adc1.read_u16())

        fullstring = str(adcstr0) + "," + str(adcstr1) + '\r'

        sys.stdout.write(fullstring)

    def input_flush(a):
        global poll_obj
        poll_result = poll_obj.poll(0)
        while (poll_result != []):
            poll_result = poll_obj.poll(0)
            dump = sys.stdin.readline()
        return
    
    emergencyTim = machine.Timer(period = 100, mode = machine.Timer.PERIODIC, callback = timee)
    #FSR_timer = machine.Timer(period = 100, mode = machine.Timer.PERIODIC, callback = send_FSR)
    input_flush_timer = machine.Timer(period = 150, mode = machine.Timer.PERIODIC, callback = input_flush)
    
    while True:
        poll_result = poll_obj.poll(5)
        wdt.feed()
        if (poll_result == []):
            continue
        data = sys.stdin.readline().strip("\n")

        # messages 1, 0, 5, 6, 7 are all used for the manipulator (numbers)
        # science messages will be all letters, not car
        # sudo udevadm info -a /dev/ttyACM0
        # need to use 1 piece from first info then 1 other, entire block
        #.install udevrules script?

        if(data == '5'):
            led.value(1)
            pca.set_claw_dir_open(i2c)
            if(activeFlagClaw):
                activeFlagClaw = True
                continue
            else:
                activeFlagClaw = True
                pwm.ramp_claw_speed(claw_pwm)
            
        elif(data == '6'):
            led.value(0)
            pwm.stop_claw(claw_pwm)
            activeFlagClaw = True
        elif(data == '7'):
            led.value(1)
            pca.set_claw_dir_close(i2c)
            if(activeFlagClaw):
                activeFlagClaw = True
                continue
            else:
                activeFlagClaw = True
                pwm.ramp_claw_speed(claw_pwm)

        elif(data == '1'):
            led.value(1)
            solenoid_pin.value(1)
            activeFlagSolenoid = True
        elif(data == '0'):
            led.value(0)
            solenoid_pin.value(0)
            activeFlagSolenoid = True

        elif(data=='reset'):
            machine.reset()
            
        else:
            led.value(0)
            pwm.stop_claw(claw_pwm)
            solenoid_pin.value(0)
    
    






#pca_0 handles the solenoid and claw with the manipulation payload, set all pins to output, default values are 1, 
#1 on a sleep pin means the device is active though









    
    



