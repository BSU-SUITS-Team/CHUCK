import serial
import serial.tools.list_ports


class MicrocontrollerInterface:
    def __init__(self, port=None, baudrate=9600, timeout=.1) -> None:
        if port is None:
            port = load_port()
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = serial.Serial(port, self.baudrate, timeout=self.timeout)

    def send(self, command):
        self._serial.write(command)
        self._serial.flush()

    def has_recieved(self):
        return self._serial.in_waiting != 0
    
    def get_recieved(self):
        return self._serial.readline().decode()
    
    def is_open(self):
        return self._serial.is_open
    
    def close(self):
        self._serial.close()

    def reopen(self):
        self._serial.open()

def save_port(port) -> None:
    with open("./port", "wt") as f:
        f.write(port)

def load_port() -> str:
    with open("./port", "rt") as f:
        return f.read().strip()

def start_test(port):
    print("binding to port", port)
    microcontroller = MicrocontrollerInterface(port)
    while True:
        q = bytearray([int(input("Enter command: "))])
        microcontroller.send(q)
        while microcontroller.has_recieved():
            print(microcontroller.get_recieved())

if __name__ == "__main__":
    print("Connected devices:")
    for i, port in enumerate(serial.tools.list_ports.comports()):
        print(f"{i}: {port.description}")
    print("Select device:")
    port = serial.tools.list_ports.comports()[int(input())]
    print("Saving port")
    save_port(port.device)


