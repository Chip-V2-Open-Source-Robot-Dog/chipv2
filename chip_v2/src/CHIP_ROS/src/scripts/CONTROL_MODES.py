'''
This class will allow us to create control modes that we can
type without remembering numbers! 
'''
from enum import IntEnum

class CONTROL_MODE(IntEnum):
    TEST = -2
    ERROR = -1
    STAND_SIT = 0
    WALK = 1
    DANCE = 2
    
