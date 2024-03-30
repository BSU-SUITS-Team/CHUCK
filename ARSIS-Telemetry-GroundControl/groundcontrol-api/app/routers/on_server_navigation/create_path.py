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
            # "type": self.type,
            "points": self.points,
        }
        return to_return

    def as_json(self):
        to_return_as_json = self.get_dict
        return json.dumps(to_return_as_json)


class CreatePoint:
    def __init__(self, x, y, lat, lon, altitude=None):
        self.x = x
        self.y = y
        self.lat = lat
        self.lon = lon
        self.altitude = altitude

    def get_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "lat": self.lat,
            "lon": self.lon,
            "altitude": self.altitude,
        }
