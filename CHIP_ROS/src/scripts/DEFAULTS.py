'''
This class will allow us to store default parameters of the system!
'''

class DEFAULTS():
    #These limits are now all taken care of in 
    #the ROBORIO Side of the platform.
    Y_LIMIT_LOWER = 0.15
    Y_LIMIT_UPPER = 0.55
    X_LIMIT_LOWER = -0.2
    X_LIMIT_UPPER = 0.2
    Z_LIMIT_LOWER = -0.1
    Z_LIMIT_UPPER = 0.1
    #these guys might yet be useful
    Y_PID_LIMIT = 0.25
    X_PID_LIMIT = 0.15
    Z_PID_LIMIT = 0.1

    #origin defaults
    PLATFORM_ORIGIN = [0.29, 0.15] #this is the locatin of the IMU everything is in ref to that.

    #NT Defaults
    NT_DEFAULT_VAL = -1000000.0

    #IP Addresses
    RIO_IP = "10.20.20.2"
    JETSON_IP = "10.20.20.12"

    #IMU Defaults
    DESIRED_RPY = [0.0, 0.0, 0.0]
    IMU_ON = False

    #leg defaults
    BIAS = 0.045
    LOW = 0.47
    HIGH = 0.49
    ZPOS_L = 0.0
    ZPOS = 0.0
    HOME = [0.0+BIAS, 0.15, ZPOS_L, 0.0+BIAS, 0.15, ZPOS_L, 0.0+BIAS, 0.15, ZPOS_L, 0.0+2*BIAS, 0.15, ZPOS_L]
    STAND = [0.0+BIAS, LOW, ZPOS,  0.0+BIAS, LOW, ZPOS, 0.0+BIAS, HIGH, ZPOS, 0.0+2*BIAS, HIGH, ZPOS]
    WALK_HOME = [0.0+BIAS, HIGH, ZPOS,  0.0+BIAS, HIGH, ZPOS, 0.0+BIAS, HIGH, ZPOS, 0.0+2*BIAS, HIGH, ZPOS]
    SIT = HOME

    #walking defaults
    #possibly won't need these dudes either
    MAX_GATE = -0.15
    MAX_STRAFE = 0.05
    MAX_SPIN = 0.05