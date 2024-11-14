from pydantic import BaseModel


class Header(BaseModel):
    # timestamp of the Redis message
    timestamp: float = 0.0
    # information of the Redis message
    information: str = ""
