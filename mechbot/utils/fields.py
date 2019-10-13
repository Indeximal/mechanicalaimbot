import enum


class CsgoTeamEnum(enum.Enum):
    TERRORISTS = "t"
    COUNTER_TERRORISTS = "ct"


@enum.unique
class DeviceStatusEnum(enum.Enum):
    INITIALIZED = enum.auto()
    AWAITING_CONNECTION = enum.auto()
    CONNECTED = enum.auto()
    CALIBRATING = enum.auto()
    CALIBRATED = enum.auto()
    TARGET = enum.auto()