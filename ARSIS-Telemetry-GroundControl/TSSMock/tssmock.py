import requests
import os
import time
import random

# Rockyard Center Coordinates in UTM
rockYardUTM = [298419.87, 3272419.59]

server = os.environ["TSS_ENDPOINT"]


def modifyXY(last_posxy):
    return last_posxy + random.uniform(-0.5, 0.5)


def modifyHeading(last_heading):
    return (last_heading + 1) % 360


def sendRequest(data):
    try:
        requests.post(url=server, data={data[0]: data[1]})
    except requests.exceptions.RequestException as e:
        print(e)


rovx = 1.0
rovy = 1.0
eva1x = 10.0
eva1y = 10.0
eva1heading = 0.0
eva2x = 20.0
eva2y = 20.0
eva2heading = 180.0


def main():
    dataXrov = ["rover_posx", rovx]
    dataYrov = ["rover_posy", rovy]
    dataXeva1 = ["imu_eva1_posx", eva1x]
    dataYeva1 = ["imu_eva1_posy", eva1y]
    dataHeadeva1 = ["imu_eva1_heading", eva1heading]
    dataXeva2 = ["imu_eva2_posx", eva2x]
    dataYeva2 = ["imu_eva2_posy", eva2y]
    dataHeadeva2 = ["imu_eva2_heading", eva2heading]
    xandys = [
        dataXrov,
        dataYrov,
        dataXeva1,
        dataYeva1,
        dataXeva2,
        dataYeva2,
    ]
    headings = [dataHeadeva1, dataHeadeva2]
    try:
        print("EV1 application started on " + server + "\n")
        while True:
            for data in xandys:
                data[1] = modifyXY(data[1])
                sendRequest(data)
            for data in headings:
                data[1] = modifyHeading(data[1])
                sendRequest(data)
            time.sleep(1.0)

    except (KeyboardInterrupt):
        print("Application closed!")


if __name__ == "__main__":
    main()
