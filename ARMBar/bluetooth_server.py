import bluetooth

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

bluetooth.advertise_service(
    server_sock,
    "CHUCKBluetooth",
    service_classes=[bluetooth.SERIAL_PORT_CLASS],
    profiles=[bluetooth.SERIAL_PORT_PROFILE]
)

print(f"Waiting for connection on RFCOMM channel {port}")

client_sock, client_info = server_sock.accept()
print(f"Accepted connection from {client_info}")

try:
    while True:
        message = input("Send command: ")
        client_sock.send(message)

except KeyboardInterrupt:
    print("Disconnected")

client_sock.close()
server_sock.close()
#This to initiate the bluetooth script for the bluetooth connection to the holo lens
#sudo apt update
#sudo apt install bluetooth bluez bluez-tools python3-bluez
#sudo systemctl enable bluetooth
#sudo systemctl start bluetooth
#bluetoothctl
#remember to add the terminal code for the raspbery pi first
