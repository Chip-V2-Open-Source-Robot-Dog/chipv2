'''
This class will allow us to store default parameters of the system!
'''

class DEFAULTS():
    Y_LIMIT_LOWER = 0.15
    Y_LIMIT_UPPER = 0.55
    X_LIMIT_LOWER = -0.2
    X_LIMIT_UPPER = 0.2
    Z_LIMIT_LOWER = -0.1
    Z_LIMIT_UPPER = 0.1

    PLATFORM_ORIGIN = [0.29, 0.15]

    NT_DEFAULT_VAL = -1000000.0

    RIO_IP = "10.20.20.2"
    JETSON_IP = "10.20.20.12"

    DESIRED_RPY = [0.0, 0.0, 0.0]
    HOME = [0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0]
    STAND = [0.0, 0.45, 0.0, 0.0, 0.45, 0.0, 0.0, 0.45, 0.0, 0.0, 0.45, 0.0]
    SIT = [0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0]