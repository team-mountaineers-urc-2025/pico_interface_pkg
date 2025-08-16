import sys
from machine import I2C
import hardware

pca0_addr = 0x20
pca1_addr = 0x21
pca2_addr = 0x22

port0_output = 0x02
port1_output = 0x03

port0_config_addr = 0x06
port1_config_addr = 0x07

#The PCAs accessed by the pico are GPIO expanders
#@address: the address for the particular PCA board to be reached through I2C
#@pinStatus: indicates whether a pin on a particular PCA is configured as output (0) or input (1), currently everything is used as output?
#@port0: raw data for specific pins on port 0 of a PCA
#@port1: ^ for port 1 though
#Not sure about the config stuff yet
class PCA:
    def __init__(self, address):
        self.address = address
        self.pinStatus = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.port0 = bytearray(1)
        self.port0[0] = 0xFF
        self.port1 = bytearray(1)
        self.port1[0] = 0xFF

#three PCAs on the payload pcb to communicate with
pca0 = PCA(address=pca0_addr)
pca1 = PCA(address=pca1_addr)
pca2 = PCA(address=pca2_addr)


#Will configure all the pins on a given pca to output, also needs the i2c object created earlier
#might want to add a return condition for unsuccessful comm?
def pca_init_all_out(pca: PCA, i2c: I2C):
    packet = bytearray(3)
    packet[0] = port0_config_addr
    packet[1] = 0x00
    packet[2] = 0x00

    i2c.writeto(pca.address, packet)

    for i in range(len(pca.pinStatus)):
        pca.pinStatus[i] = 0


#Sets the value of a specified pin (that should be an output) on a specified
def pca_set_pin(pca: PCA, pin: int, i2c: I2C):
    if((pin > 15) or (pca.pinStatus[pin] == 1)):
        sys.stdout.write("pin check failed set")
        return -1
    
    packet  = bytearray(2)
    
    if(pin < 8):
        #update internal account of pin levels
        pca.port0[0] = pca.port0[0] | (0x01 << pin) 
         #command portion of packet, changing the output of a pin on port 0
        packet[0] = port0_output
         #what to change the pins to
        packet[1] = pca.port0[0]
        
    elif(pin < 16):
        pca.port1[0] = pca.port1[0] | (0x01 << (pin-8))
        packet[0] = port1_output
        packet[1] = pca.port1[0]

    i2c.writeto(pca.address, packet)
       


def pca_clear_pin(pca: PCA, pin: int, i2c: I2C):
    if((pin > 15) or (pca.pinStatus[pin] == 1)):
        sys.stdout.write("pin check failed clear")
        return -1
    
    packet = bytearray(2)

    if(pin < 8):
        pca.port0[0] = pca.port0[0] & ~(0x01 << pin)
        packet[0] = port0_output
        packet[1] = pca.port0[0]
        
    if(pin < 16):
        pca.port1[0] = pca.port1[0] & ~(0x01 << (pin-8))
        packet[0] = port1_output
        packet[1] = pca.port1[0]

    i2c.writeto(pca.address, packet)
        


#Claw needs to open and close
def set_claw_dir_open(i2c: I2C):
    pca_clear_pin(pca0, 8, i2c)
    
def set_claw_dir_close(i2c: I2C):
    pca_set_pin(pca0, 8, i2c)


