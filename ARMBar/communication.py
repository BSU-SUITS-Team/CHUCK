import requests

def send_to_hololens(key):
    requests.post("http://hololens-ip/input", json={"key": key})
def handle_input(button_name):
    print(f"Sending {button_name}")
    client_sock.send(button_name)