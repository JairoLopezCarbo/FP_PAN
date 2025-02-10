import math
import geopy.distance
import numpy as np

EARTH_RADIUS = 6371000
MAX_RADIUS = 10
METERSROUND = 10  #configParam   X,X m
ANGLEROUND  = 4  #configParam   XX  º

class Point:
    def __init__(self, lat, lon, alt = 0, *, time = 0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.time = time
        self.HSpeed, self.VSpeed = 0, 0
        self.HAccel, self.VAccel = 0, 0
        self.bearing = 0
        self.maxRadius = MAX_RADIUS

    def Copy(self):
        newPoint = Point(self.lat, self.lon, self.alt, self.time)
        return newPoint
    

#BASICS
def LatLonDistance(pt1, pt2): #meters[m]
    coords_1 = (pt1.lat, pt1.lon)
    coords_2 = (pt2.lat, pt2.lon)  
    return round(geopy.distance.geodesic(coords_1, coords_2).km*1000, METERSROUND)

def RealDistance(pt1, pt2): #meters[m]
    horizontalDist = LatLonDistance(pt1, pt2)
    verticalDist = pt2.alt-pt1.alt
    realDist = math.sqrt(math.pow(horizontalDist,2)+ math.pow(verticalDist, 2))
    return round(realDist, METERSROUND)


def BearingAngle(pt1, pt2): #angle[º] 0 to 360
    # if pt1.lon==pt2.lon and pt1.lat==pt2.lat:
    #     return None
    y = math.sin((pt2.lon-pt1.lon)*math.pi/180)*math.cos(pt2.lat*math.pi/180)
    x = math.cos(pt1.lat*math.pi/180)*math.sin(pt2.lat*math.pi/180)-math.sin(pt1.lat*math.pi/180)*math.cos(pt2.lat*math.pi/180)*math.cos((pt2.lon-pt1.lon)*math.pi/180)
    return round((math.atan2(y, x)*180/math.pi+360) % 360, ANGLEROUND)

def AttackAngle(pt1, pt2): #angle[º] -90 to 90
    x = LatLonDistance(pt1,pt2)
    y = pt2.alt-pt1.alt
    return round(math.atan2(y, x)/math.pi*180, ANGLEROUND)


def DestinyPoint(origin, distance, bearing, *, deltaAltitude = 0, attackAngle = 0):
    if deltaAltitude != 0 and attackAngle != 0:
        raise ValueError("You can only specify either deltaAltitude or attackAngle, not both.")
    elif bearing == None:
        return Point(origin.lat, origin.lon, origin.alt + np.sin(attackAngle*np.pi/180)*distance)
    elif attackAngle != 0:
        deltaAltitude = distance*np.sin(attackAngle*np.pi/180)
        distance = distance*np.cos(attackAngle*np.pi/180)
        
        
        
    originLat = origin.lat*math.pi/180
    originLon = origin.lon*math.pi/180
    delta = distance/EARTH_RADIUS

    destLat = math.asin(math.sin(originLat)* math.cos(delta)+ math.cos(originLat)*math.sin(delta)*math.cos(bearing*math.pi/180))
    destLon = (originLon+math.atan2(math.sin(bearing*math.pi/180)*math.sin(delta)*math.cos(originLat), math.cos(delta)-math.sin(originLat)*math.sin(destLat)))
    return Point(destLat*180/math.pi, destLon*180/math.pi, origin.alt+deltaAltitude)


#UTILS 
def DistanceFromLineProjection(line, point): #Distance between line (pointList) and a point, meters[m] 
    distance = LatLonDistance(line[0], line[1])
    bearing = BearingAngle(line[0], line[1])/180*math.pi
    surfaceVector = np.array([distance*math.cos(bearing), distance*math.sin(bearing), line[1].alt-line[0].alt])
    distance = LatLonDistance(line[0], point)
    bearing = BearingAngle(line[0], point)/180*math.pi
    pt = np.array([distance*math.cos(bearing), distance*math.sin(bearing), point.alt-line[0].alt])
    
    D =-1*np.dot(surfaceVector, pt)
    mu = -1*D/np.dot(surfaceVector,surfaceVector)
    return np.sqrt(np.sum(((surfaceVector*mu)-pt)**2))

def IdentifyTurn(initPoint, turnPoint, endPoint,*, margin = 0, divide = False):

    prevBearing = BearingAngle(initPoint, turnPoint)
    prevAttack = AttackAngle(initPoint, turnPoint)

    nextBearing = BearingAngle(turnPoint, endPoint)
    nextAttack = AttackAngle(turnPoint, endPoint)
    
    deltaBearing = 0
    if prevBearing!=None and nextBearing!=None:
        if nextBearing<prevBearing:
            deltaBearing += 360-prevBearing
            prevBearing = 0
        deltaBearing += nextBearing-prevBearing
        if deltaBearing > 180:
            deltaBearing -= 360

    deltaAttack = nextAttack-prevAttack
    
    
    horizontal = "None"
    vertical = "None"
    if np.abs(deltaBearing)>margin:
        if deltaBearing > 0:
            horizontal = "Right"
        else:
            horizontal = "Left"
    else:
        deltaBearing = 0
        
    if np.abs(deltaAttack)<margin:
        if deltaAttack > 0:
            vertical = "Up"
        else:
            vertical = "Down"
    else:
        deltaAttack = 0

    if divide:
        return [horizontal, deltaBearing],[vertical, deltaAttack]
    
    elif np.abs(deltaBearing) >= 2*np.abs(deltaAttack):
        return [horizontal, deltaBearing]
    else:
        return [vertical, deltaAttack]
    
def divide(numerator, denominator):
    if numerator == 0 and denominator == 0:
        return 0
    elif denominator == 0:
        return np.Infinity
    return numerator/denominator