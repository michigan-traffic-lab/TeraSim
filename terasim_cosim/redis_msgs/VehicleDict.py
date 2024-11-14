from pydantic import BaseModel
from typing import Dict

from terasim_cosim.redis_msgs import Header, Vehicle


class VehicleDict(BaseModel):
    # header of the message
    header: Header = Header()

    # dictionary of the vehicles, with the vehicle ID as the key and the vehicle information as the value
    data: Dict[str, Vehicle] = {}
