# README

## Testing

To run the pytest suite:
```
docker-compose up --build tests
```

## Raspberry Pi Armbar Buttons

`test_scripts/armbar_gpio_server.py` runs on a Raspberry Pi and emits
`armbar_button_press` events to `groundcontrol-api` when physical buttons are
pressed.

Install the runtime dependencies on the Pi:
```
sudo apt update
sudo apt install python3-gpiozero python3-requests
```

Wire each momentary button between a BCM GPIO pin and any ground pin. The script
uses GPIO Zero's internal pull-up mode, so the button pulls the input low when
pressed. The default map is:
```
armbar 1 -> BCM GPIO 17
armbar 2 -> BCM GPIO 18
armbar 3 -> BCM GPIO 27
armbar 4 -> BCM GPIO 22
armbar 5 -> BCM GPIO 23
armbar 6 -> BCM GPIO 24
```

Run it against the API server:
```
python3 test_scripts/armbar_gpio_server.py --api http://groundcontrol-api-host:8181
```

To run it on boot, install it as a `systemd` service. Do not pass `--api` in the
service command if you want the remote API address to be changeable at runtime
from the config file.

Example `/etc/systemd/system/armbar-gpio.service`:
```
[Unit]
Description=Armbar GPIO button event sender
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/CHUCK/Telemetry-GroundControl
ExecStart=/usr/bin/python3 /home/pi/CHUCK/Telemetry-GroundControl/test_scripts/armbar_gpio_server.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable it:
```
sudo systemctl daemon-reload
sudo systemctl enable --now armbar-gpio.service
```

When the service is already running, the script acts as a companion command
instead of opening the GPIO pins again. This updates `/etc/default/armbar-gpio`,
follows the service logs, and leaves the service running when you detach:
```
sudo python3 test_scripts/armbar_gpio_server.py --remote-ip 192.168.1.50
```

The service reads that config file before each button event, so the next button
press uses the new API address without restarting the service. Press `Ctrl-C` to
detach from the logs; the script will print that `armbar-gpio.service` is still
running.

Use `--no-log-follow` to update the config and return immediately:
```
sudo python3 test_scripts/armbar_gpio_server.py --remote-ip 192.168.1.50 --no-log-follow
```

Override the pin map when the wiring differs:
```
python3 test_scripts/armbar_gpio_server.py \
  --api http://groundcontrol-api-host:8181 \
  --map 1:5,2:6,3:13,4:19,5:26,6:21
```

References:
- GPIO Zero `Button` API: https://gpiozero.readthedocs.io/en/stable/api_input.html#button
- GPIO Zero installation: https://gpiozero.readthedocs.io/en/stable/installing.html
- Raspberry Pi GPIO header: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio-and-the-40-pin-header
