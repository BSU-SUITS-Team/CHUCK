#!/bin/python3
import requests
import argparse
import time
import logging

logging.basicConfig(level=logging.NOTSET)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("user_id")
    args = parser.parse_args()
    heading = 0
    while True:
        heading += 1
        heading %= 360
        data = {"latitude": 0, "longitude": 0, "altitude": 0, "heading": heading}
        logging.info(data)
        headers = {"Content-Type": "application/json", "accept": "application/json"}

        r = requests.post(
            f"http://0.0.0.0:8080/location/{args.user_id}/update_location",
            json=data,
            headers=headers,
        )
        logging.info(r.content)

        time.sleep(1)


if __name__ == "__main__":
    main()
