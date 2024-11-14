from pydantic import BaseModel
from typing import Dict

from terasim_cosim.redis_msgs import Header, Metric


class MetricDict(BaseModel):
    # header of the message
    header: Header = Header()

    # dictionary of the metric information, with the metric ID as the key and the metric information as the value
    data: Dict[str, Metric] = {}
