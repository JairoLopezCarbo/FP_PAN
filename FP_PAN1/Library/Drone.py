from Library.LatLonProjection import *
import numpy as np

class Drone:
    def __init__(self, horizonalSpeed, verticalSpeed, horizontalAccel, verticalAccel, maxRadius = 10):
        self.horizontalSpeed = horizonalSpeed
        self.verticalSpeed = verticalSpeed
        self.horizontalAccel = horizontalAccel
        self.verticalAccel = verticalAccel
        self.maxRadius = maxRadius
        
    def GetMaxSpeed(self, radius, deltaBearing):
        
        maxRadius = self.maxRadius*np.abs(deltaBearing)/180
        if radius < maxRadius: 
            return (radius/maxRadius)*self.horizontalSpeed, self.verticalSpeed
        else:
            return self.horizontalSpeed, self.verticalSpeed
        
        
        