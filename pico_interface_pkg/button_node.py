import rclpy, serial, time
from rclpy.node import Node
from std_msgs.msg import Float32



class PicoWriter(Node):
    
    def __init__(self):
        
        super().__init__('pico_led_node')

        self.location = '3'

        self.port = serial.Serial(
            port="/dev/ttyACM0", 
            parity=serial.PARITY_EVEN, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=1
        )

        self.temp_pub = self.create_publisher(
            msg_type=Float32,
            topic= '/pico/pico_button',
            qos_profile=10,
            )
        
        self.timer = self.create_timer(1.0,self.get_data)
         

    def get_data(self):
        pico_response = self.port.read_until().strip().decode()
        print(pico_response + '\r')
        if(pico_response):
            self.temp_pub.publish(Float32(data=float(pico_response.strip())))

    def activate_pico(self):
        print('calling activate')
        print(self.location)
        self.port.write(f"{self.location}\r".encode())  
        


def main(args=None):
    print("initializing")
    rclpy.init(args=args)

    led_pico = PicoWriter()
    led_pico.activate_pico()

    print("spin led")
    rclpy.spin(led_pico)
    led_pico.destroy_node()
    time.sleep(2)

    rclpy.shutdown()
    print("shutdown succesful")


if __name__ == '__main__':
    main()