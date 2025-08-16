import rclpy, serial, time
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from robot_interfaces.msg import STargetedFloat
from robot_interfaces.msg import STargetedBool




#Might need to add a lot of try, except to serial port stuff to prevent crashing in the event of a
#brief disconnection
class PicoWriter(Node):
    
    def __init__(self):
        
        super().__init__('pico_science_node')

        self.declare_parameter('pico_path', '/dev/urc/mc/pi_pico')
        self.pico_path = self.get_parameter('pico_path').get_parameter_value().string_value
        self.moist = 0.0
        self.temp = 0.0
        self.bl = False

        while True:        
            try:
                print("Trying to connect to Pico")
                self.port = serial.Serial(
                    #Will need a udev rule for the device eventually, might need another python package to interpret symlink
                    #Coprocessor is always ttyACM0, pixhawk is 1 so pico is ttyACM2 for now
                    port=self.pico_path, 
                    parity=serial.PARITY_EVEN, 
                    stopbits=serial.STOPBITS_ONE, 
                    timeout=0.4
                )
                break
            except serial.SerialException:
                continue

        self.picoActuatorSub = self.create_subscription(
            STargetedFloat,
            '/science/actuator_pos',
            self.send_to_pico_act,
            10
        )

        self.picoRelaySub = self.create_subscription(
            STargetedBool,
            '/science/relay_status',
            self.send_to_pico_relay,
            10
        )


        self.picoTempPub = self.create_publisher(
            msg_type=Float32,
            topic= 'picoTemp',
            qos_profile = 10
            )
        
        self.picoMoisturePub = self.create_publisher(
            msg_type=Float32,
            topic= 'picoMoisture',
            qos_profile = 10
            )
        
        self.picoClawSubscriber2 = self.create_subscription(
            String,
            '/manipulator/testing',
            self.send_to_pico2,
            10
        )

        self.data_timer = self.create_timer(3.5,self.get_temp_moist)
         

    def get_data(self):
        try:
            self.get_logger().info("Getting data from Pico")
            pico_response = self.port.readline().decode().strip('\n')
        except serial.SerialException:
            self.start_pico_connection_blocking()
            return
        except serial.SerialTimeoutException:
            return
        
        #self.get_logger().info(f"Received: {pico_response}")
        return pico_response
        #self.temp_pub.publish(Float32(data=float(pico_response.split("Temp:")[1].strip())))

    def get_temp_moist(self):
        data = self.get_data()
        self.port.reset_input_buffer()
        try:
            moisture_s, temp_s = data.split(",")
            self.moist = float(moisture_s)
            self.temp = float(temp_s)
        except Exception as e:
            self.get_logger().info('pico thing')
        self.picoMoisturePub.publish(Float32(data=self.moist))
        self.picoTempPub.publish(Float32(data=self.temp))




    def send_to_pico2(self, msg):
        try:
            self.get_logger().info("yep")
            self.port.write(f"{msg.data}\n".encode())
        except serial.SerialException:
            self.start_pico_connection_blocking()
        except serial.SerialTimeoutException():
            return
        

    def send_to_pico_act(self, msg):
        try:
            if msg.data < 0.0 or msg.data > 1.0:
                self.get_logger().error(f"Error, extension value {msg.data} for {msg.target} not in valid range of 0.0 - 1.0")
                return
            self.get_logger().info((msg.target + "\n" + str(msg.data) + "\n").encode())
            self.port.write((msg.target + "\n" + str(msg.data) + "\n").encode())
        except serial.SerialException:
            self.start_pico_connection_blocking()
        except serial.SerialTimeoutException:
            return


    def send_to_pico_relay(self, msg):
        try:
            if ((msg.target == "hb") and (msg.data == True)):
                self.port.write(("hbon\n").encode())
            elif ((msg.target == "hb") and (msg.data == False)):
                self.port.write(("hboff\n").encode())
            elif ((msg.target == "u2d2") and (msg.data == True)):
                self.port.write(("u2d2on\n").encode())
            elif ((msg.target == "u2d2") and (msg.data == False)):
                self.port.write(("u2d2off\n").encode())
        except serial.SerialException:
            self.start_pico_connection_blocking()
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