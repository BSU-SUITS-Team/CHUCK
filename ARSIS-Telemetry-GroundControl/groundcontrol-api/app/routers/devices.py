from asyncio import Queue
import json
from fastapi import APIRouter
import requests
import re
import ipaddress


router = APIRouter(prefix="/devices", tags=["devices"])

devices = {}
commands = {}


@router.get("/get_configured")
async def return_all_configured_devices():
    """This function returns a list of all configured devices.

    Returns:
        (dict): A dict of all configured devices.
    """
    return devices


@router.get("/is_configured/{name}")
async def return_configured_device(name: str):
    """This function returns whether or not a device is configured.

    Args:
        name (str): The name of the device to be checked.

    Returns:
        (bool): True if the device is configured, False if it is not."""
    return name in devices.keys()


@router.post("/configure/{name}")
async def configure_device(name: str, ip_address: str):
    """This function configures a device to be controlled by ground control.

    Args:
        name (str): The name of the device to be configured. This can be arbitrary.
        ip_address (str): The ip address of the device to be configured. This is used to send commands to the device.

    Returns:
        (int): The status code of the request. 200 if the device was configured successfully, 400 if the ip address was invalid or the device was already configured.
    """
    if ip_address is None:
        return 400
    try:
        ip_address = ipaddress.ip_address(ip_address)
        devices[name] = ip_address
        commands[name] = Queue()
    except ValueError:
        return 400
    return 200


@router.delete("/remove/{name}")
async def remove_device(name: str):
    """This function removes a device from the list of configured devices.

    Args:
        name (str): The name of the device to be removed.

    Returns:
        (int): The status code of the request. 200 if the device was removed successfully, 400 if the device was not configured.
    """
    if name in devices.keys():
        del devices[name]
        del commands[name]
        return 200
    return 400


@router.post("/broadcast")
async def broadcast_command(command: str):
    """This function sends a command to all configured devices.

    Args:
        command (str): The command to be sent to the devices. This is transmitted as a byte string.

    Returns:
        (int): The status code of the request. 200 if the command was sent successfully.
    """
    for device in devices:
        commands[device].put(command)
    return 200


@router.post("/send_command")
async def send_command(names: list[str], command: str):
    """This function sends a command to a list of configured devices.

    Args:
        names (list[str]): A list of the names of the devices to send the command to.
        command (str): The command to be sent to the devices. This is transmitted as a byte string.

    Returns:
        (int): The status code of the request. 200 if the command was sent successfully, 404 if one or more of the devices were not configured.
    """
    for name in names:
        if name in devices.keys():
            commands[name].put(command)
    return 200


@router.post("/send_command/{name}")
async def send_command(name: str, command: str):
    """This function sends a command to a configured device.

    Args:
        name (str): The name of the device to send the command to.
        command (str): The command to be sent to the device. This is transmitted as a byte string.

    Returns:
        (int): The status code of the request. 200 if the command was sent successfully, 404 if the device was not configured.
    """
    if name in devices.keys():
        commands[name].put(command)
        return 200
    return 404

@router.get("/get_commands/{name}")
async def get_commands(name: str):
    """This function gets the commands for a configured device.
    It is used by the device to get commands from ground control.

    Args:
        name (str): The name of the device to get commands for.
    """
    if name in devices.keys():
        return commands[name].get()
    return 404

@router.get("/has_commands")
async def has_commands(name: str):
    """This function gets whether or not a configured device has commands.
    It is used by the device to get commands from ground control.

    Args:
        name (str): The name of the device to get commands for.
    """
    if name in devices.keys():
        return not commands[name].empty()
    return 404
