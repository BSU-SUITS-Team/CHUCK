import requests
import socket
import multiprocessing
import json

HOST = '0.0.0.0'
UNITY_PORT = 5000
ROVER_PORT = 5001

key_listeners = {}

global_send_queue = multiprocessing.Queue()


def execute_key_listener(data: dict) -> None:
    """
    This function is used to execute the key listeners that are registered.

    Key listeners are functions that are registered to be executed when a certain key is recieved from the rover or unity.
    Their argument is the json object that is recieved from the rover or unity.

    Args:
        data: The json object that is recieved from the rover or unity.
    """
    for key in key_listeners:
        if key in data.keys():
            key_listeners[key](data[key])


def parse_unity_instructions(data: str) -> dict:
    """
    This function is used to parse the data that is recieved from the unity and rover.

    Args:
        data: The data that is recieved from the unity and rover.
    """
    return json.loads(data)


def DataEvaluator(queue: multiprocessing.Queue) -> None:
    """
    This function is used to evaluate the data that is recieved from the rover and unity.
    """
    fragment = b""
    while True:
        if not queue.empty():
            data = queue.get()
            if fragment == b'':
                fragment = data
            else:
                fragment = fragment + data
                for end_indx in range(len(fragment)):
                    try:
                        decoded = json.loads(
                            fragment[:end_indx].decode('utf-8'))
                        execute_key_listener(decoded)
                        fragment = fragment[end_indx:]
                    except json.decoder.JSONDecodeError:
                        pass


def UnityServer() -> None:
    """
    This function is used to start the unity server.

    This is different from the DeviceServer as the unity server only recieceves commands from unity
    which are used to control the rover. Currently no data is sent back to unity.
    """
    print('Unity server started')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, UNITY_PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            multiprocessing.Process(
                target=Connection, args=(conn, addr)).start()


def DeviceServer() -> None:
    """
    This function is used to start the rover server.

    This is different from the Unity Server as ut sends instructions to the 
    rovers/vision kits and recieves the telemetry data from connected devices.
    """
    print('Rover server started')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, ROVER_PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            multiprocessing.Process(
                target=Connection, args=(conn, addr)).start()


def Connection(conn, addr) -> None:
    connection_send_queue = multiprocessing.Queue()
    recieved_data_queue = multiprocessing.Queue()
    parser = multiprocessing.Process(
        target=DataEvaluator, args=(recieved_data_queue,))
    with conn:
        print('New connection from ', addr)
        data = b''
        while True:
            data += conn.recv(1024)
            if not data:
                break
            recieved_data_queue.put(data)
            to_send = {}
            while not global_send_queue.empty():
                to_send.update(global_send_queue.get())
            while not connection_send_queue.empty():
                to_send.update(connection_send_queue.get())
            conn.sendall(json.dumps(to_send).encode('utf-8'))
    parser.terminate()


### ACTUAL FUNCTIONALITY ###
def register_device_id(id: dict) -> None:
    """
    This function is used to register the device id of the rover/vision kit.

    This is used to identify the rover when it is connected to the ground control server and to relay commands.
    """
    


if __name__ == '__main__':
    unity_process = multiprocessing.Process(target=UnityServer)
    rover_process = multiprocessing.Process(target=DeviceServer)

    # Start the processes and wait for them to finish. This should take forever as the processes should never end.
    unity_process.start()
    rover_process.start()
    unity_process.join()
    rover_process.join()
