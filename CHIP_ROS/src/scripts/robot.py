'''
this is a class that constructs a robot
'''
import component

class robot:
    '''
    components is a list of components 
    '''
    def __init__(self, components):
        self.components=components

    def findCoG(self):
        XC = 0.0
        ZC = 0.0
        MTOTAL = 0.0

        for component in self.components:
            XC+=component.x*component.mass
            ZC+=component.z*component.mass
            MTOTAL+=component.mass
        
        return [XC/MTOTAL, ZC/MTOTAL]
