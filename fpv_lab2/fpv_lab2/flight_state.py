from enum import Enum, auto


class FlightState(Enum):
    WAITING_FOR_SYSTEM = auto()
    PREARM_CHECK = auto()
    SETTING_GUIDED = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    MOVING_TO_TARGET = auto()
    LANDING = auto()
    WAITING_FOR_LANDING = auto()
    FINISHED = auto()
    ERROR = auto()
