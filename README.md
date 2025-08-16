# pico_interface_pkg
<p>This package includes the code for both the node that needs to run on the Latte Panda to communicate with the pico as well as the code that needs to be flashed onto the pico.</p>

## Nodes
### Arm Node
### Science Node

## Pico Code
<p>The code on the pico itself has a simple overall function. It will poll its serial port, take in a message, and then decode that message and exectute an action based on its contents. </p>


## To Do
- [x] Write a script to auto flash the pico
- [x] Basic ros node and two-way communication
- [ ] PWM servo control for 5v motors (GPIO 2-7) (See Payload PCB v3.1 - Servos)
- [ ] PWM for H bridge motor drivers (GPIO 8-15) (See Payload PCB v3.1 - Motor Drivers)
- [ ] I2C Interfacing to PCA9555s (See Payload PCB v3.1 - I2C GPIO Expanders)
  - [ ] Expander 1 - motor direction control (12v motors)
  - [ ] Expander 2 - motor driver faults and payload select pins
  - [ ] Expander 3 - LEDs for mission 
- [ ] Analog Recieves (GPIO 26-28)

## Starting
```bash
ros2 run pico_interface_pkg pico_host_node
```


# pico_comm_claw.py
  FSR relevant information:
    2 important pulishers, FSR 0 is associated with the FSR on pin 26 on the board and FSR 1 is with the FSR on pin 27

    Topics: /pico/FSR0 and /pico/FSR1 which are the dedicated topics for FSR 0 and 1 output (for display ourposes on GUI)

    FSR methods: input - none from the node, constantly receiving data from the PICO as a string in the form of FSR0_resistance,FSR1_resistance where at no force upon the FSRs the reading will be 65535,65535 and at maximum force will be 0,0. These readings are output to the ROS2 topis. Should the timing of the reading be desynced with the PICO and is only able to read a partial line, the function will not publish anything and instead log a timing warning. This should only happen once or twice on launch or very rarely during operation.

    Currently the timer is set to run once per second, should this need to be faster you can replace the 1.0 in the timer creation with a lower number but it should be noted that if you do this you will need to alter the code in main.py on the PICO to match this timing in order to ensure desync between the writing and reading is at a minimum



# button_node.py
  A node which is only relevant for early prototyping. Can be removed with no issue. Used a basic circuit to turn on an led bulb when a button was pushed and sent the status of the button/led back to the pico then to this node.

# host_node.py
  A node which is only relevant for early prototyping. Can be removed with no issue. Used a basic circuit with a temperature sensor to communicate data between the PICO and this node.

# led_node.py
 A node which is only relevant for early prototyping. Can be removed with no issue. Used a basic circuit to intermittently turn on and off an led and communicated the led status between the  PICO and this node.

# old_host.py
  A node which is only relevant for early prototyping. Can be removed with no issue. Was used as a simple echo between the node and the PICO

# main.py
  This file will need to be flashed onto the PICO before operation. Runs micropython instead of standard python.

  send_FSR() - reads the 16bit inputs from the ADCs on pin 26 and 27 and formats them into a string in the form of ADC26,ADC27 where ADC26 and 27 will be numbers indicating the force on the FSRs. A higher number indicated less force with 65536 being the baseline with no force and 0 being maximum force. After formatting the string is then written to the associated node inside pico_comm_claw.py 

  Other relevant code - FSR_timer declaration creates a watchdog timer which repeatedly calls the send_FSR() function to ensure constant reading of the ADCs. To cause this to run more or less often change the period argument which is currently set to run once per second.

# hardware.py
  This file will need to be flashed onto the PICO before operation. Runs micropython instead of standard python.

  This file is primarily a setup file used to initiate various motors and machines which will be interfaced with the PICO.

  fsr_init() - initializes 2 variables as ADCs reading from pin 26 and pin 27 then returns them to be read from in main.py