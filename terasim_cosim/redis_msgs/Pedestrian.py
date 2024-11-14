from pydantic import BaseModel


class Pedestrian(BaseModel):
    # Acceleration
    ## longitudinal acceleration of the pedestrian (meters per second squared)
    accel_long: float = 0.0
    ## lateral acceleration of the pedestrian (meters per second squared)
    accel_lat: float = 0.0

    # Position
    ## x position of the pedestrian in the UTM coordinate system (meters)
    x: float = 0.0
    ## y position of the pedestrian in the UTM coordinate system (meters)
    y: float = 0.0
    ## elevation of the pedestrian (meters)
    z: float = 0.0

    # Orientation
    ## orientation of the pedestrian, ranging from -pi to pi, where 0 means the pedestrian is heading to the east, pi/2 means the vehicle is heading to the north (radians)
    orientation: float = 0.0

    # Size (https://www.baua.de/DE/Angebote/Publikationen/AWE/AWE108.pdf?__blob=publicationFile)
    ## length of the pedestrian (meters)
    length: float = 0.215
    ## width of the pedestrian (meters)
    width: float = 0.478
    ## height of the pedestrian (meters)
    height: float = 1.719

    # Speed
    ## longitudinal speed of the pedestrian (meters per second)
    speed_long: float = 0.0
    ## lateral speed of the pedestrian (meters per second)
    speed_lat: float = 0.0

    # additional information of the pedestrian
    additional_information: str = ""
