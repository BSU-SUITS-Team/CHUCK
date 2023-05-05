from .rover_path import rover_path

paths = [rover_path]
paths_json = {p.name: p.as_json() for p in paths}
