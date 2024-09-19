from .create_path import CreatePath, CreatePoint

point1 = CreatePoint("point 1", 43.600618831356776, -116.19761557353348, 2600)
point2 = CreatePoint("point 1", 43.60095850212377, -116.19691720521107, 2600)
rover_path = CreatePath("Rover Path")
rover_path.set_as_path()
rover_path.add_point(point1)
rover_path.add_point(point2)
