'''
component.py is a class that is an object
that represents the individual parts of the robot
'''

class component:
    '''
    It stores the componet's position with respect to an origin and mass
    '''
    def __init__(self, x, z, mass, origin):
        self.x = x-origin[0]
        self.z = z-origin[1]
        self.mass = mass

