from Library.LatLonProjection import *
from Library.Drone import *
import numpy as np

ACCURACY= 1000
MIN_RADIUS = 2


class Discretize:
    def __init__(self, flightPlan, drone):
        self.flightPlan = {"traces":[], "time":[0]}
        start = 0 
        for i in range(len(flightPlan)):
            if flightPlan[i].time != 0:
                self.flightPlan["traces"].append(flightPlan[start:i])
                self.flightPlan["time"].append(flightPlan[i].time)
                start = i+1
        self.flightPlan["traces"].append(flightPlan[start:])
        self.drone = drone
    
    def SetTurns(self, turnCoefficientList):
        self.distance, self.time = [], []
        self.traces = []
        distance, time = 0, 0
        for flightPlan in self.flightPlan["traces"]:
            trace = []
            lastPoint, speed =  flightPlan[0], 0
            for i in range(1, len(flightPlan)-1):
                turn = Turn(self.drone, lastPoint, flightPlan[i], flightPlan[i+1], turnCoefficientList[i-1])
                trace.append(Straight(self.drone, lastPoint, turn.initPoint, speed))
                speed = turn.speed
                while not(trace[(i-1)*2].SetEndSpeed(speed)):
                    speed = trace[(i-1)*2].initSpeed
                    trace[(i-1)*2-1].SetSpeed(speed)
                    i-=1
                    
                lastPoint, speed = turn.endPoint, trace[-1].endSpeed
                turn.SetSpeed(speed)
                trace.append(turn)
                    
            trace.append(Straight(self.drone, lastPoint, flightPlan[-1], speed))
            i, speed = len(flightPlan)-1,0
            while not(trace[(i-1)*2].SetEndSpeed(speed)):
                    speed = trace[(i-1)*2].initSpeed
                    trace[(i-1)*2-1].SetSpeed(speed)
                    i=-1
            self.traces.append(trace)
            self.distance.append([distance])
            self.time.append([self.flightPlan["time"][len(self.time)]+ time])
            for i in range(len(trace)):
                self.distance[-1].append(self.distance[-1][i]+trace[i].GetDistance())      
                self.time[-1].append(self.time[-1][i]+trace[i].GetTime()) 
            distance, time = self.distance[-1][-1], self.time[-1][-1]
          
                
    
    def GetPoints(self, *, distance = None, time = None):
        Points =  []
        i, j = 0, 0
        
        if distance!= None:
            for d in range(int(self.distance[-1][-1]//distance)+1):
                d = d*distance
                while not(d >= self.distance[j][i] and d < self.distance[j][i+1]):
                    if i < len(self.distance[j])-2:
                        i+=1
                    else:
                        i = 0
                        j += 1
                        
                Points.append(self.traces[j][i].GetPoint(distance=d-self.distance[j][i]))
                Points[-1].time +=  self.time[j][i]
        elif time!= None:
            for t in range(int(self.time[-1][-1]//time)+1):
                t = t*time
                while not(t >= self.time[j][i] and t < self.time[j][i+1]) and t > self.time[j][0]:
                        if i < len(self.time[j])-2:
                            i+=1
                        else:
                            i = 0
                            j += 1
                            if t < self.time[j][0]:
                                break
                if t >= self.time[j][0]:
                    Points.append(self.traces[j][i].GetPoint(time=t-self.time[j][i]))
                else:
                    if t < self.time[j-1][-1]+(self.time[j][0]-self.time[j-1][-1])/2:
                        Points.append(self.traces[j-1][-1].GetPoint()[1].Copy())
                    else:
                        Points.append(self.traces[j][0].GetPoint()[0].Copy())
                    Points[-1].FPLvel = "Hover"
                
                Points[-1].time =  t
        else:
            j = 0
            for trace in self.traces:
                i=0
                for t in trace:
                    Points.append(t.GetPoint()[0].Copy())
                    Points[-1].time = self.time[j][i]
                    i+=1
                j+=1
        
        finalPoint = self.flightPlan["traces"][-1][-1]
        finalPoint.time = self.time[-1][-1]
        finalPoint.horizontalSpeed, finalPoint.verticalSpeed = 0,  0
        finalPoint.horizontalAccel, finalPoint.verticalAccel = 0,  0
        finalPoint.FPLvel = "Straight"
        Points.append(finalPoint)
        return Points
        
    

class Turn:
    def __init__(self, drone, initPoint, turnPoint, endPoint, turnCoefficient):
        #Set given data
        self.drone = drone
        self.initPoint = initPoint
        self.turnPoint = turnPoint
        self.endPoint = endPoint
        
        #Angle calculation
        self.initBearing, self.initAttack = BearingAngle(initPoint, turnPoint), AttackAngle(initPoint, turnPoint)
        self.endBearing,  self.endAttack  = BearingAngle(turnPoint, endPoint),  AttackAngle(turnPoint, endPoint)
        self.totalTurn = np.arccos(np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)
                                   +np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)
                                   +np.sin(self.initAttack*np.pi/180)*np.sin(self.endAttack*np.pi/180))*180/np.pi

        
        #Turn Radius 
            # 1) Calculation
        radius = min([RealDistance(turnPoint, initPoint), RealDistance(turnPoint, endPoint)])*np.tan(np.pi/2-self.totalTurn*np.pi/360)
        if turnPoint.maxRadius < radius:
            radius = turnPoint.maxRadius 
        self.R = turnCoefficient*radius
        
            # 2) Decisions
        if self.R >= MIN_RADIUS:
            self.m = self.R/np.sin(np.pi/2-self.totalTurn*np.pi/360)
            D = self.R/np.tan(np.pi/2-self.totalTurn*np.pi/360)
            if RealDistance(turnPoint, initPoint) > D:
                self.initPoint = initPoint.Copy()
                newPoint = DestinyPoint(turnPoint, D, BearingAngle(turnPoint, initPoint), attackAngle=AttackAngle(turnPoint, initPoint))
                self.initPoint.lat, self.initPoint.lon, self.initPoint.alt = newPoint.lat, newPoint.lon, newPoint.alt
            if RealDistance(turnPoint, endPoint) > D:
                self.endPoint = turnPoint.Copy()
                newPoint = DestinyPoint(turnPoint, D, BearingAngle(turnPoint, endPoint), attackAngle=AttackAngle(turnPoint, endPoint)) 
                self.endPoint.lat, self.endPoint.lon, self.endPoint.alt = newPoint.lat, newPoint.lon, newPoint.alt
            self.x_0 = int(-np.cos(self.totalTurn*np.pi/360)*D * 10**8) / 10.0**8
            self.totalDistance = self.R*(np.arcsin(-self.x_0/self.R)-np.arcsin(self.x_0/self.R))
            
            
            
            angle = (180-self.totalTurn)*np.pi/360
            attack = np.arcsin((-np.sin(self.initAttack*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))*180/np.pi
            bearing = 180/np.pi*np.arctan2((-np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180),
                                        (-np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))
            self.rotationPoint = DestinyPoint(turnPoint, self.m, bearing, attackAngle=attack)
            
            deltaBearing, deltaAttack =IdentifyTurn(initPoint, turnPoint, endPoint, margin = 0, divide = True)
            maxHSpeed, maxVSpeed = self.drone.GetMaxSpeed(radius, deltaBearing[1], deltaAttack[1])
            self.speed = min([self.initPoint.FPLvel, self.turnPoint.FPLvel])
            for i in range(0, ACCURACY+1):
                angle = np.arctan(-self.x_0/np.sqrt(self.R**2-self.x_0**2))*(2*i/ACCURACY)*180/np.pi
                _, s_attack = self.GetSpeedDirection(angle)
                _, a_attack = self.GetAccelDirection(angle)
                
                self.speed = min([self.speed]+[divide(maxHSpeed, np.cos(s_attack*np.pi/180)), divide(maxVSpeed, np.abs(np.sin(s_attack*np.pi/180)))]+ [np.sqrt(divide(self.R*self.drone.horizontalAccel,np.cos(a_attack*np.pi/180))),np.sqrt(divide(self.R*self.drone.verticalAccel,np.abs(np.sin(a_attack*np.pi/180))))])

        else:
            self.R, self.m = 0, 0 
            self.initPoint, self.endPoint = self.turnPoint, self.turnPoint
            self.totalDistance, self.x_0 = 0, 0
            self.speed = 0
            
        
                                                                    
    def GetSpeedDirection(self, angle):
        attack = np.arcsin((np.sin(self.initAttack*np.pi/180)*np.sin((self.totalTurn-angle)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((self.totalTurn)*np.pi/180))*180/np.pi
        bearing = 180/np.pi*np.arctan2((np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((self.totalTurn)*np.pi/180),
                                       (np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((self.totalTurn)*np.pi/180))
        return bearing, attack
    
    def GetAccelDirection(self, angle):
        x = self.R*np.tan(np.pi/180*(angle-np.arctan(self.x_0/np.sqrt(self.R**2-self.x_0**2))*180/np.pi))/np.sqrt(np.tan(np.pi/180*(angle-np.arctan(self.x_0/np.sqrt(self.R**2-self.x_0**2))*180/np.pi))**2+1)
        y = -np.sqrt(np.abs(self.R**2-x**2))+self.m
        angle = (np.arctan(x/y)+(180-self.totalTurn)*np.pi/360)*180/np.pi
        attack = np.arcsin((-np.sin(self.initAttack*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))*180/np.pi
        bearing = 180/np.pi*np.arctan2((-np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180),
                                       (-np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))
        position = DestinyPoint(self.turnPoint, np.sqrt(x**2+y**2), bearing, attackAngle=attack)
        return BearingAngle(position, self.rotationPoint), AttackAngle(position, self.rotationPoint)
        
    def GetDistance(self, x= None):
        if x== None:
            return self.totalDistance
        elif x>= self.x_0 and x<=-self.x_0:
            distance = self.R*(np.arcsin(divide(x,self.R))-np.arcsin(divide(self.x_0,self.R)))
            return distance
        else:
            return 0
        
    def GetTime(self, x=None):
        if x== None:
            return divide(self.GetDistance(),self.speed)
        elif x>= self.x_0 and x<=-self.x_0:
            return divide(self.GetDistance(x),self.speed)
        else:
            return 0
    
    def SetSpeed(self, newSpeed):
        if newSpeed<self.speed:
            self.speed = newSpeed 
            
                
    def GetPoint(self, *, distance = None, time = None):
        if distance == None and time == None:
            return [self.initPoint, self.endPoint]
        elif time!= None:
            distance = time*self.speed
        
        
        x = self.R*np.sin(distance/self.R+np.arcsin(self.x_0/self.R))
        y = -np.sqrt(self.R**2-x**2)+self.m
        
        angle = (np.arctan(x/y)+(180-self.totalTurn)*np.pi/360)*180/np.pi
        attack = np.arcsin((-np.sin(self.initAttack*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))*180/np.pi
        bearing = 180/np.pi*np.arctan2((-np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180),
                                       (-np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-angle)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(angle*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))
        
        point = DestinyPoint(self.turnPoint, np.sqrt(x**2+y**2), bearing, attackAngle=attack)
        point.time = self.GetTime(x)
        
        angle = np.arctan(x/np.sqrt(self.R**2-x**2))*180/np.pi-np.arctan(self.x_0/np.sqrt(self.R**2-self.x_0**2))*180/np.pi
        point.bearing, s_attack = self.GetSpeedDirection(angle)
        point.HSpeed, point.VSpeed = self.speed*np.cos(s_attack*np.pi/180), self.speed*np.sin(s_attack*np.pi/180)
        
        _, a_attack = self.GetAccelDirection(angle)
        point.HAccel, point.VAccel = self.speed**2*np.cos(a_attack*np.pi/180), self.speed**2*np.sin(a_attack*np.pi/180)
        point.FPLvel = "Turn"
        # point.HAccel, point.VAccel = self.speed**2*attack, self.speed**2*self.accelDirection[i][1]
        return point
        

class Straight: 
    def __init__(self, drone, initPoint, endPoint, initSpeed):
        self.drone = drone
        
        self.initPoint = initPoint
        self.endPoint = endPoint
        self.initSpeed = initSpeed 
        self.totalDistance = RealDistance(initPoint, endPoint)
        if self.totalDistance != 0:
            self.bearing, self.attack = BearingAngle(initPoint, endPoint), AttackAngle(initPoint, endPoint)
            self.maxAccel = min([divide(drone.horizontalAccel,np.cos(self.attack*np.pi/180)), np.abs(divide(drone.verticalAccel,np.sin(self.attack*np.pi/180)))])
        
    
    def GetDistance(self):
        return self.distance[-1]
    
    def GetTime(self):
        return self.time[-1]
    
    def GetEndSpeed(self):
        return self.endSpeed 
          
    def SetEndSpeed(self, endSpeed): 
        self.endSpeed = endSpeed
        self.distance = [0]
        self.time = [0]
        self.speed = [self.initSpeed]
        
        
        valid = True
        
        if self.totalDistance == 0:
            self.acceleration =  [0]
            if self.endSpeed < self.initSpeed:
                self.initSpeed = self.endSpeed
                self.speed = [self.initSpeed]
                self.acceleration = [0]
                valid = False
            else:
                self.endSpeed = self.initSpeed
        
        
        elif np.abs(endSpeed**2-self.initSpeed**2)/(2*self.maxAccel) >= self.totalDistance:
            self.acceleration =  [self.maxAccel]
            if self.endSpeed < self.initSpeed:
                self.initSpeed = np.sqrt(endSpeed**2+2*self.maxAccel*self.totalDistance)
                self.speed = [self.initSpeed]
                self.acceleration = [-self.maxAccel]
                
                valid = False
            else:
                self.endSpeed = np.sqrt(self.initSpeed**2+2*self.maxAccel*self.totalDistance)

            
        else:
            self.acceleration =  [self.maxAccel]
            maxSpeed = min([self.initPoint.FPLvel , divide(self.drone.horizontalSpeed,np.cos(self.attack*np.pi/180)), np.abs(divide(self.drone.verticalSpeed,np.sin(self.attack*np.pi/180)))])
            if maxSpeed == 0:
                print("a")
            if (maxSpeed**2-max([self.initSpeed ,self.endSpeed])**2)/(2*self.maxAccel) < (self.totalDistance-np.abs(endSpeed**2-self.initSpeed**2)/(2*self.maxAccel))/2:
                self.distance.append((maxSpeed**2-self.initSpeed**2)/(2*self.maxAccel))
                self.time.append((maxSpeed-self.speed[-1])/self.acceleration[-1])
                self.speed.append(maxSpeed)
                self.acceleration.append(0)

                self.distance.append(self.totalDistance-(self.endSpeed**2-maxSpeed**2)/(2*-self.maxAccel))
                self.time.append(self.time[-1]+(self.distance[-1]-self.distance[-2])/maxSpeed)
                self.speed.append(maxSpeed)
                self.acceleration.append(-self.maxAccel)
            else:
                maxSpeed = np.sqrt(2*self.maxAccel*(self.totalDistance-np.abs(endSpeed**2-self.initSpeed**2)/(2*self.maxAccel))/2+max([self.initSpeed, self.endSpeed])**2)
                self.distance.append((maxSpeed**2-self.initSpeed**2)/(2*self.maxAccel))
                self.time.append((maxSpeed-self.speed[-1])/self.acceleration[-1])
                self.speed.append(maxSpeed)
                self.acceleration.append(-self.maxAccel)
            
        self.distance.append(self.totalDistance)
        self.time.append(self.time[-1]+divide((self.endSpeed-self.speed[-1]),self.acceleration[-1]))
        self.speed.append(self.endSpeed)
        self.acceleration.append(0)    
            
        return valid
    
    def GetPoint(self, *, distance = None, time = None):
        i = 0
        list =  []
        value = None
        if distance!= None:
            value = distance
            list = self.distance
        elif time!= None:
            value = time
            list = self.time
        else:
            return [self.initPoint, self.endPoint]
            
        while list[i+1] < value:
            i+= 1
            if i == len(list):
                return None
        
        if distance!= None:
            if self.acceleration[i] == 0:
                time = self.time[i] + (distance-self.distance[i])/self.speed[i]
            else:
                time = self.time[i]+(-self.speed[i]+np.sqrt(self.speed[i]**2+2*self.acceleration[i]*(distance-self.distance[i])))/(2*self.acceleration[i])
        elif time != None:
            distance = self.distance[i]+self.speed[i]*(time-self.time[i])+self.acceleration[i]*(time-self.time[i])**2/2
        
        speed = self.speed[i]+self.acceleration[i]*(time-self.time[i])
        point = DestinyPoint(self.initPoint, distance, self.bearing, attackAngle=self.attack)
        point.time = time
        point.HSpeed, point.VSpeed = speed*np.cos(self.attack*np.pi/180), speed*np.sin(self.attack*np.pi/180)
        point.HAccel, point.VAccel = self.acceleration[i]*np.cos(self.attack*np.pi/180), self.acceleration[i]*np.sin(self.attack*np.pi/180)
        point.bearing = self.bearing
        point.FPLvel = "Straight"
        return point
        
        
                
        
        
    
    
    
            
            
            