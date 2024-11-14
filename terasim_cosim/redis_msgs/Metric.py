from pydantic import BaseModel


class Metric(BaseModel):
    # whether the metric should alert
    alert: bool = False

    # value of the metric
    metric_value: float = 0.0

    # additional information of the metric
    info: str = ""
