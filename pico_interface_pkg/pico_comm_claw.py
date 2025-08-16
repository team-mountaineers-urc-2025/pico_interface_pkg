import rclpy, serial, time
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import Joy




#Might need to add a lot of try, except to serial port stuff to prevent crashing in the event of a
#brief disconnection
class PicoWriter(Node):
    
    def __init__(self):
        
        super().__init__('pico_claw_communicator_node')
        self.declare_parameter('pico_path', '/dev/urc/mc/pi_pico')
        self.pico_path = self.get_parameter('pico_path').get_parameter_value().string_value
        self.bl = False



        while True:        
            try:
                self.port = serial.Serial(
                    port=self.pico_path, 
                    parity=serial.PARITY_EVEN, 
                    stopbits=serial.STOPBITS_ONE, 
                    timeout=0.1,
                    write_timeout=0,
                )
                break
            except serial.SerialException:
                continue

        self.picoClawSubscriber = self.create_subscription(
            Joy,
            '/manipulator/joy', #should be the correct topic, raw outputs from controller, we only care about buttons
            self.send_to_pico,
            10
        )


        self.picoSolenoidControl = self.create_subscription(
            Bool,
            'solenoid_control', #should be the correct topic, raw outputs from controller, we only care about buttons
            self.send_to_pico_sol,
            10
        )
    
        self.picoClawSubscriber2 = self.create_subscription(
            String,
            '/manipulator/testing',
            self.send_to_pico2,
            10
        )
        


        self.FSR0_pub = self.create_publisher(
            msg_type=Float32,
            topic= '/pico/FSR0',
            qos_profile=10,
            )
        
        self.FSR1_pub = self.create_publisher(
            msg_type=Float32,
            topic= '/pico/FSR1',
            qos_profile=10,
            )
         
#        self.timer = self.create_timer(1.0,self.get_fsr)

    def send_to_pico2(self, msg):
        try:
            self.get_logger().info("yep")
            self.port.write(f"{msg.data}\n".encode())
        except serial.SerialException:
            self.start_pico_connection_blocking()
        except serial.SerialTimeoutException:
            return

    def get_fsr(self):
        try:
            pico_response = self.port.readline().strip().decode()

            fsrfull = str(pico_response).split(",")

            fsr0 = fsrfull[0]
            fsr1 = fsrfull[1]
            self.FSR0_pub.publish(Float32(data=float(fsr0)))
            self.FSR1_pub.publish(Float32(data=float(fsr1)))
        except:
            self.get_logger().info('bad timing on fsr, passing')
            pass

    def send_to_pico(self, msg):
        self.half_send_tracker = False
        #I believe that the x and b button status are elements 1 and 3 in the buttons int32 array
        #A button for solenoid
        aData = msg.buttons[0]

        #X and B buttons for the claw
        bData = msg.buttons[1]
        xData = msg.buttons[2]

        try:

            #it might not be the best way to comm with the pico, but I'll just go with this sequential messaging for now
            #tell the solenoid to activate
            if (aData == 1):
                sendSolenoid = '1'
                self.port.write(f"{sendSolenoid}\n".encode())

            #if none of the buttons are pressed, just tell the claw to stop
            if((bData == 0) and (xData == 0)):
                sendClaw = '6'
                return
            #if both buttons are pressed, just tell the claw to stop
            elif((bData == 1) and (xData == 1)):
                sendClaw = '6'
                self.port.write(f"{sendClaw}\n".encode())
                return
            #just x button pressed at this point
            elif(xData == 1):
                sendClaw = '5'
                self.port.write(f"{sendClaw}\n".encode())
                self.get_logger().info("Opening the Claw")
                return
            #just b button pressed at this point
            elif(bData == 1):
                sendClaw = '7'
                self.port.write(f"{sendClaw}\n".encode())
                self.get_logger().info("Closing the Claw")
                return
            #if buttons not recognized, send a stop message
            else:
                sendClaw = '6'
                self.port.write(f"{sendClaw}\n".encode())
                return
            
        except serial.SerialException:
            self.start_pico_connection_blocking()
            return

        except serial.SerialTimeoutException:
            return
        

    def send_to_pico_sol(self, msg):
        if (msg.data == True):
            sendSolenoid = '1'
        else:
            sendSolenoid = '0'
        
        try:
            self.port.write(f"{sendSolenoid}\n".encode())
        except serial.SerialException:
            self.start_pico_connection_blocking()
            return
        except serial.SerialTimeoutException:
            return
        
    def start_pico_connection_blocking(self):
        if(self.bl == False):
            self.bl = True
            while True:
                try:
                    self.port = serial.Serial(
                        port=self.pico_path, 
                        parity=serial.PARITY_EVEN, 
                        stopbits=serial.STOPBITS_ONE, 
                        timeout=0.1,
                        write_timeout=0
                    )
                    self.bl = False
                    return
                except serial.SerialException:
                    continue
        else:
            return



        
        





        


def main(args=None):
    print("initializing")
    rclpy.init(args=args)
    pico_comm = PicoWriter()
    rclpy.spin(pico_comm)
    pico_comm.destroy_node()
    time.sleep(2)

    rclpy.shutdown()
    print("shutdown succesful")


if __name__ == '__main__':
    main()
