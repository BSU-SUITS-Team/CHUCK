# Robot Code
## Description
Allows the TCP Control of an Arduino from Unity. It has two main parts.
- First, It has a configuration menu that allows the arduino to be configured to automatically connect to a network, and attempt to connect to the C# server.
- Seond it has the motor control/runtime part of the program that controls how the arduino manages the input that it recieves from the Unity program.

The main goal of the program is to allow the easy configuration and control of the robot in an extendable and potentially configuragable way.

## Useage

### Setup
First, download the new [Arduino IDE 2.0 beta](https://www.arduino.cc/en/software) and install it. Next, open the arudino IDE and install the board configuration for the Arduino Nano 33 IOT. Then, make sure that the Atmel-ICE bootloader is selected.

### Compilation
If the firmware is out of date or needs to be reflashed for some reason, connect the arduino to a laptop that has the Arduino IDE and the sketch.ino file. And then click the Verify and Upload button near the top left. You should only see one warning in the console, regarding the initalization of a char pointer. 

### Configuration
After the firmware is flashed to the arduino, connect it to the USB cable. The orange and yellow led lights should turn on. These are what the led lights mean for the status of the firmware.

- yellow light: the device has power
- orange light: the device is waiting for external input/updates (Serial connection, wifi connection, etc..)

Because the firmware is new, and has no saved configuration it will wait indefinately for a serial connection. (Once we save the settings it will only wait for 1s on future startups before it attempts to use those settings.) We can now open the serial monitor (Ctrl+Shift+M) and we should see a menu appear on the terminal that looks somthing like this:

```
Configure Device:
w - Change wifi info
p - Print Network Host IP Address
t - Change IP target
n - Print network name
c - Connect to network
s - Save configuration
r - Read configs
q - Exit Setup

>
```

#### `Change Wifi Info`
This will prompt the user to enter the name and password of the wifi network that it should connect to/

#### `Print Network Host IP Address`
This will attempt to connect to the currently configured network (See [Change Wifi Info]()) and then print the IP address of the device hosting the network.

#### `Change IP target`
This sets the IP address that the arduino will try to connect to (The Unity server) after it has tried to connect to the network. It needs for be formatted as 4 triplets of numbers. (ex. `127.134.9.12` would be entered `127.134.009.012`)

#### `Print network name`
Prints the name of the currently loaded network.

#### `Connect to Network`
This will test connecting to the network. It is almost exactly like what it does when it is not conencted to the debug monitor. Except that it does not actually process the recieved packets. It will instead print the recieved data to the console where it can be inspected.

#### `Save configuration`
This will save the configuration of the device and will disable farther communication. DO NOT DISCONNECT THE DEVICE UNTIL THE ORANGE LIGHT TURNS OFF.

#### `Read configs`
Loads the currently saved network and IP target info. The stored data is deleted every time the firware is updated.

#### `Exit Setup`
Just ends the setup process.

To setup the rover for usage first enter `w` to set the wifi info. Then find the IP address of the target device. Next run `t` and enter that IP address. It will need for be formatted as 4 triplets of numbers. (ex. `127.134.9.12` would be entered `127.134.009.012`) Then enter `s` to save and press the reset button on the arduino to reset it again. If you want at this point, the terminal should auto-reconnect, and you should be able to read the settings and verify that it was configured correctly. Then CLOSE ALL SERIAL MONITORS, if any are open it will go back into debug mode, and reset the arduino again. It should now try to connect the the Network and the Unity Program that it was programmed to connect to. 