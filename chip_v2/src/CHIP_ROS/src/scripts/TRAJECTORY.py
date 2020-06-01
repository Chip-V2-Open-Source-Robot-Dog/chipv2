'''
Trajectory is a static class that allows us to create a trajectory
object that stores the XYZ foot positions that we want to travel 
through and then the functions handle adding points to the trajectory 
and then running through while accounting for the current
foot positions. 
'''
from DEFAULTS import DEFAULTS

class TRAJECTORY():
    '''constructor for the class creates a trajectorry runner object'''
    def __init__(self):
        self.waypoints = []
        self.saved = DEFAULTS.STARTING_CONFIG
    
    '''allows you to add a waypoint to the class'''
    def addWaypoint(self, point):
        self.waypoints.append(point)

    '''allows you to add more than one waypoint'''
    def addWaypoints(self, points):
        self.waypoints = self.waypoints+points

    '''handle trajectory running'''
    def tick(self, current_pos):
        if(len(self.waypoints)>0):
            self.saved = self.waypoints[0]
            if(self.equals(current_pos, self.waypoints[0], DEFAULTS.EPSILON)):
                self.saved = self.waypoints.pop(0)
        return self.saved

    '''clears any waypoints in the trajectory'''
    def clear(self, pos):
        self.waypoints = []
        self.saved = pos

    '''static method that determines if list values differ by less than epsilon'''
    def equals(self, list1, list2, epsilon):
        print("__________________")
        print(list1)
        print(list2)
        bools = []
        list3 = zip(list1, list2)
        for item in list3:
            diff = abs(item[0]-item[1])
            bools.append(diff<=epsilon)
        print(list3)
        print(bools)
        return bools==[True]*len(list3)




'''HOW TO USE'''
'''
traj = new TRAJECTORY
TRAJECTORY.addWaypoint(waypoint)

TRAJECTORY.tick(/ADD THE CURRENT POSITIONS)
'''


