import serial

port = serial.Serial(
    port="/dev/ttyACM0", 
    parity=serial.PARITY_EVEN, 
    stopbits=serial.STOPBITS_ONE, 
    timeout=1)


def main():

    while True:
        port.flush()

        input_str = input('Input number')
        port.write(f"{input_str}\r".encode())
        mes = port.read_until().strip()
        print(mes.decode())


if __name__ == "__main__":
    main()