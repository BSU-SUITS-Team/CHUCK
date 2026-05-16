import requests

def send_to_hololens(key):
    requests.post("http://hololens-ip/input", json={"key": key})