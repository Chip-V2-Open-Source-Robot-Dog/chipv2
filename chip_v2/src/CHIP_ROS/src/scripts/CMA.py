'''
This is a CMA class that gives us the direction the feet
should move given a desired leg movement
'''
from DEFAULTS import DEFAULTS
import math
from LEG_MODEL import MODEL as legModel

class CMA():
    #____________________________________________________________________________________________________________________________________________________________________
    # THESE METHODS ARE CLASS METHODS
    #____________________________________________________________________________________________________________________________________________________________________

    '''constructor for the class creates a CMA calculator object'''
    def __init__(self):
        #take the CMA_CENTER command and convert to XYZ
        self.CENTER = DEFAULTS.CMA_CENTER
        self.PREVIOUS = 0

    '''feedforward method of CMA that actually returns the command we should send
    CMD - is the command we want to send
    LEG - is the leg we want to MOVE (SINGULAR FOR NOW), 0, 1, 2, or 3 for FLL FRL BLL BRL in that order
    RAW is if the CMD is RAW or not true for raw and false for not raw

    RETURNS COMMAND IN FORMAT YOU INPUTTED IT IN (RAW OR XYZ)
    '''
    def ff(self, CMD, RAW, LEG):
        #first convert a raw command to xyz
        CMD_NEW = []
        THETA_1s = []
        if (RAW):
            global CMD_NEW
            global THETA_1s
            a = self.calculateTHETAS( [CMD[0], CMD[1], CMD[2]] )
            b = self.calculateTHETAS( [CMD[3], CMD[4], CMD[5]] )
            c = self.calculateTHETAS( [CMD[6], CMD[7], CMD[8]] )
            d = self.calculateTHETAS( [CMD[9], CMD[10], CMD[11]] )
            L1 = self.forwardsKinematics( a )
            L2 = self.forwardsKinematics( b )
            L3 = self.forwardsKinematics( c )
            L4 = self.forwardsKinematics( d )
            CMD_NEW = L1+L2+L3+L4
            THETA_1s = [a[0], b[0], c[0], d[0]]
        else:
            global CMD_NEW
            global THETA_1s
            a = self.inverseKinematics(CMD[0], CMD[1], CMD[2])
            b = self.inverseKinematics(CMD[3], CMD[4], CMD[5])
            c = self.inverseKinematics(CMD[6], CMD[7], CMD[8])
            d = self.inverseKinematics(CMD[9], CMD[10], CMD[11])
            CMD_NEW = CMD
            THETA_1s = [a[0], b[0], c[0], d[0]]

        #okay so now we have the commands as XYZ commands and we have the leg THETA_1s
        #now let's remember these aren't the current they're the DESIRED and all we're doing is OFFSETTING THE X_D from the center
        #FOR NON MOVING LEGS so first we calculate the offsets 
        THETA_M = THETA_1s[LEG]
        dCOG = -5.0/4.0*math.cos(THETA_M-math.pi/4.0)/100.0 #in the opposite direction! because reverse coord frame
        dX = dCOG/3.0+math.copysign(0.005, dCOG) #this is how much in the x-direction each leg needs to move from the CENTER POSITION plus a 0.005 safety factor

        if(LEG==2 or LEG==3):
            global dX
            dX = -5.0*dX

        #now we need to send the feet to the x_CMD in question
        CMD_RETURN = []
        if (LEG==0):
            global CMD_RETURN
            CMD_RETURN = [CMD_NEW[0], CMD_NEW[1], CMD_NEW[2], dX, CMD_NEW[4], CMD_NEW[5], dX, CMD_NEW[7], CMD_NEW[8], dX, CMD_NEW[10]-0.04, CMD_NEW[11]]
        if (LEG==1):
            global CMD_RETURN
            CMD_RETURN = [dX, CMD_NEW[1], CMD_NEW[2], CMD_NEW[3], CMD_NEW[4], CMD_NEW[5], dX, CMD_NEW[7]-0.04, CMD_NEW[8], dX, CMD_NEW[10], CMD_NEW[11]]
        if (LEG==2):
            global CMD_RETURN
            CMD_RETURN = [dX, CMD_NEW[1], CMD_NEW[2], dX, CMD_NEW[4]-0.05, CMD_NEW[5], CMD_NEW[6], CMD_NEW[7], CMD_NEW[8], dX, CMD_NEW[10], CMD_NEW[11]]
        if (LEG==3):
            global CMD_RETURN
            CMD_RETURN = [dX, CMD_NEW[1]-0.05, CMD_NEW[2], dX, CMD_NEW[4], CMD_NEW[5], dX, CMD_NEW[7], CMD_NEW[8], CMD_NEW[9], CMD_NEW[10], CMD_NEW[11]]
        
        self.PREVIOUS = LEG
        #return the value
        return self.doIK(CMD_RETURN)

    #____________________________________________________________________________________________________________________________________________________________________
    # THESE METHODS ARE HELPER METHODS
    #____________________________________________________________________________________________________________________________________________________________________

    """Bounds the value input between lower and upper, known limits of the system, think of them as virtual hardstops"""
    def clip(self, value, lower, upper):
        if (value>=upper):
            return upper
        if (value<=lower):
            return lower
        return value

    """actual inverse kinematic model/formula"""
    def inverseKinematics(self, xD, yD, zD):
        l1 = legModel.L3
        l2 = legModel.L6

        rSquared = xD*xD+yD*yD

        #setup limits for rsquared and z
        rSquared = self.clip(rSquared, (l1-l2)*(l1-l2), (l1+l2)*(l1+l2))
        zD = self.clip(zD, -0.1, 0.1)

        print(rSquared)
        print(zD)

        phi = math.atan2(yD, xD)
        kneeTheta = math.acos((rSquared-l1*l1-l2*l2)/(-2.0*l1*l2))
        alpha = math.acos((l2*l2-l1*l1-rSquared)/(-2.0*l1*math.sqrt(rSquared)))
        shoulderTheta = phi-alpha
        hingeTheta = math.asin(zD/yD)

        return [shoulderTheta, hingeTheta, kneeTheta]

    """calcualte the commands to send"""
    def calculateCMDS(self, thetas):
        return [thetas[0] * 100.0/(2*math.pi), thetas[1] * 100.0/(2*math.pi), thetas[2] * 100.0/(2*math.pi)]

    """calcualte the thetas from the positions"""
    def calculateTHETAS(self, cmds):
        return [cmds[0] * (2*math.pi)/100.0, cmds[1] * (2*math.pi)/100.0, cmds[2] * (2*math.pi)/100.0]

    """actual inverse kinematic model/formula"""
    def forwardsKinematics(self, thetas):
        #define some needed variables
        l1 = legModel.L3
        l2 = legModel.L6
        t1 = thetas[0]
        t2 = thetas[1]
        t3 = thetas[2]
        #caluclate X and Y
        xC = l1*math.cos(t1)-l2*math.cos(t3-t1)
        yC = l1*math.sin(t1)+l2*math.sin(t3-t1)
        #calculate Z
        zC = yC*math.sin(t2)
        #return
        return [xC, yC, zC]
    
    '''
    This method takes in a xyz setpoint and outputs the CMDS
    '''
    def doIK(self, data):
        #recieve the XYZ data for a leg
        xFL = data[0]
        yFL = data[1]
        zFL = data[2]
        #perform inverse kinematics and get CMDS
        FLCMDS = self.calculateCMDS(self.inverseKinematics(xFL, yFL, zFL))

        #recieve the XYZ data for a leg
        xFR = data[3]
        yFR = data[4]
        zFR = data[5]
        #perform inverse kinematics and get CMDS
        FRCMDS = self.calculateCMDS(self.inverseKinematics(xFR, yFR, zFR))

        #recieve the XYZ data for a leg
        xBL = data[6]
        yBL = data[7]
        zBL = data[8]
        #perform inverse kinematics and get CMDS
        BLCMDS = self.calculateCMDS(self.inverseKinematics(xBL, yBL, zBL))

        #recieve the XYZ data for a leg
        xBR = data[9]
        yBR = data[10]
        zBR = data[11]
        #perform inverse kinematics and get CMDS
        BRCMDS = self.calculateCMDS(self.inverseKinematics(xBR, yBR, zBR))

        #populate the commands vector and return it
        return FLCMDS+FRCMDS+BLCMDS+BRCMDS