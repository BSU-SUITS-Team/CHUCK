import requests
import socket
import multiprocessing
import json

HOST = '0.0.0.0'
UNITY_PORT = 5000
ROVER_PORT = 5001

key_listeners = {}

recieved_data_queue = multiprocessing.Queue()
unity_information_queue = multiprocessing.Queue()
rover_information_queue = multiprocessing.Queue()


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


def EvaluateData() -> None:
    """
    This function is used to evaluate the data that is recieved from the rover and unity.
    """
    while True:
        if not recieved_data_queue.empty():
            data = recieved_data_queue.get()
            execute_key_listener(data)


def UnityServer() -> None:
    print('Unity server started')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, UNITY_PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            multiprocessing.Process(
                target=Connection, args=(conn, addr)).start()


def RoverServer() -> None:
    print('Rover server started')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, ROVER_PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            multiprocessing.Process(
                target=Connection, args=(conn, addr)).start()


def Connection(conn, addr) -> None:
    with conn:
        print('New connection from ', addr)
        data = b''
        while True:
            data += conn.recv(1024)
            if not data:
                break
        data = parse_unity_instructions(data.decode('utf-8'))
        recieved_data_queue.put(data)
        to_send = {}
        while not rover_information_queue.empty():
            to_send.update(rover_information_queue.get())
        conn.sendall(json.dumps(to_send).encode('utf-8'))


if __name__ == '__main__':
    parser_process = multiprocessing.Process(target=EvaluateData)
    unity_process = multiprocessing.Process(target=UnityServer)
    rover_process = multiprocessing.Process(target=RoverServer)

    # Start the processes and wait for them to finish. This should take forever as the processes should never end.
    parser_process.start()
    unity_process.start()
    rover_process.start()
    unity_process.join()
    rover_process.join()
    parser_process.join()
