
import rclpy, serial
from rclpy.node import Node
from std_msgs.msg import Float32

class PicoInterfaceNode(Node):

    def __init__(self):

        super().__init__('pico_interface_node')


        self.port = serial.Serial(
            port="/dev/ttyACM0", 
            parity=serial.PARITY_EVEN, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=1
        )

        self.temp_pub = self.create_publisher(
            msg_type=Float32,
            topic= '/pico/pico_temperature_c',
            qos_profile=10,
            )
        
        self.timer = self.create_timer(1.0,self.get_data)


    def get_data(self):
        input_str = '10'
        self.port.write(f"{input_str}\r".encode())
        pico_response = self.port.read_until().strip().decode()
        print(pico_response)
        self.temp_pub.publish(Float32(data=float(pico_response.split("Temp:")[1].strip())))
        

def main(args=None):

    rclpy.init(args=args)
    drivebase_node = PicoInterfaceNode()
    rclpy.spin(drivebase_node)
    drivebase_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()