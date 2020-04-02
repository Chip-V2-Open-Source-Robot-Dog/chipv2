'''
This class will allow us to create control modes that we can
type without remembering numbers! 
'''
from enum import IntEnum

class CONTROL_MODE(IntEnum):
    STAND_SIT = 0
    WALK = 1
    ERROR = -1
