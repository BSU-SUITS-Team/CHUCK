from enum import Enum
import json

class CreatePath:
    def __init__(self, name):
        self.name = name
        self.points = []

    # TODO make this an enum or something similar so its not just a string
    def set_as_path(self):
        self.type = "path"

    def add_point(self, point):
        self.points.append(point)

    def get_dict(self):
        to_return = {
            "type": self.type,
            "points": [p.get_dict() for p in self.points],
        }
        return to_return

    def as_json(self):
        to_return_as_json = self.get_dict
        return json.dumps(to_return_as_json)


class CreatePoint:
    def __init__(self, name, lat, long, altitude):
        self.name = name
        self.lat = lat
        self.long = long
        self.altitude = altitude

    def get_dict(self):
        return {
            "name": self.name,
            "lat": self.lat,
            "long": self.long,
            "altitude": self.altitude,
        }
