from .rover_path import rover_path

paths = [rover_path]
paths_json = {p.name: p.get_dict() for p in paths}
