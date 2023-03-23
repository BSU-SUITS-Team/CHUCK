import requests
import argparse
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("user_id")
    args = parser.parse_args()
    heading = 0
    while True:
        heading += 1
        heading %= 360
        print(heading)
        data = {"latitude": 0, "longitude": 0, "altitude": 0, "heading": heading}
        print(data)
        headers = {"Content-Type": "application/json", "accept": "application/json"}

        r = requests.post(
            f"http://0.0.0.0:8080/location/{args.user_id}/update_location",
            json=data,
            headers=headers,
        )
        print(r)
        print(r.content)

        time.sleep(1)


if __name__ == "__main__":
    main()
