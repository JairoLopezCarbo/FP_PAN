from Library.LatLonProjection import *
from Library.Drone import *
import numpy as np

ACCURACY= 100

class Discretize:
    def __init__(self, flightPlan, drone):
        self.flightPlan = [flightPlan[0]]
        for point in flightPlan[1:]:
            if not(point.lat == self.flightPlan[-1].lat and point.lon == self.flightPlan[-1].lon and point.alt == self.flightPlan[-1].alt):
                self.flightPlan.append(point)
        self.drone = drone
    
    def SetTurns(self, turnCoefficientList):
        self.traces = []
        lastPoint, speed =  self.flightPlan[0], 0
        for i in range(1, len(self.flightPlan)-1):
            turn = Turn(self.drone, lastPoint, self.flightPlan[i], self.flightPlan[i+1], turnCoefficientList[i-1])
            self.traces.append(Straight(self.drone, lastPoint, turn.initPoint, speed))
            speed = turn.speed
            while not(self.traces[(i-1)*2].SetEndSpeed(speed)):
                speed = self.traces[(i-1)*2].initSpeed
                self.traces[(i-1)*2-1].SetSpeed(speed)
                i-=1
                
            lastPoint, speed = turn.endPoint, self.traces[-1].endSpeed
            turn.SetSpeed(speed)
            self.traces.append(turn)
                
        self.traces.append(Straight(self.drone, lastPoint, self.flightPlan[-1], speed))
        i, speed = len(self.flightPlan)-1,0
        while not(self.traces[(i-1)*2].SetEndSpeed(speed)):
                speed = self.traces[(i-1)*2].initSpeed
                self.traces[(i-1)*2-1].SetSpeed(speed)
                i=-1
    
        self.distance, self.time = [0], [0]
        for i in range(len(self.traces)):
            self.distance.append(self.distance[i]+self.traces[i].GetDistance())      
            self.time.append(self.time[i]+self.traces[i].GetTime()) 
    
    def GetPoints(self, *, distance = None, time = None):
        Points =  []
        i = 0
        
        if distance!= None:
            for d in range(int(self.distance[-1]//distance)+1):
                d = d*distance
                while not(d >= self.distance[i] and d < self.distance[i+1]):
                    i+=1
                Points.append(self.traces[i].GetPoint(distance=d-self.distance[i]))
                Points[-1].time +=  self.time[i]
        elif time!= None:
            for t in range(int(self.time[-1]//time)+1):
                t = t*time
                while not(t >= self.time[i] and t < self.time[i+1]):
                    i+=1
                Points.append(self.traces[i].GetPoint(time=t-self.time[i]))
                Points[-1].time +=  self.time[i]
        else:
            i = 0
            for trace in self.traces:
                Points.append(trace.GetPoint()[0])
                Points[-1].time = self.time[i]
                i+=1
        
        finalPoint = self.flightPlan[-1]
        finalPoint.time = self.time[-1]
        finalPoint.horizontalSpeed, finalPoint.verticalSpeed = 0,  0
        finalPoint.horizontalAccel, finalPoint.verticalAccel = 0,  0
        Points.append(finalPoint)
        return Points
        
    

class Turn:
    def __init__(self, drone, initPoint, turnPoint, endPoint, turnCoefficient):
        self.drone = drone
        
        self.initPoint = initPoint
        self.turnPoint = turnPoint
        self.endPoint = endPoint
        
        self.initBearing, self.initAttack = BearingAngle(initPoint, turnPoint), AttackAngle(initPoint, turnPoint)
        self.endBearing,  self.endAttack  = BearingAngle(turnPoint, endPoint),  AttackAngle(turnPoint, endPoint)
        self.totalTurn = np.arccos(np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)
                                   +np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)
                                   +np.sin(self.initAttack*np.pi/180)*np.sin(self.endAttack*np.pi/180))*180/np.pi
        
        
        self.minAttack, self.maxAttack = min([np.abs(self.initAttack),np.abs(self.endAttack)]), max([np.abs(self.initAttack),np.abs(self.endAttack)])
        
        self.maxRadius = turnPoint.maxRadius
        angleAprox= 0.05
        alpha, aprox = self.totalTurn*np.pi/360, self.totalTurn*np.pi/180*angleAprox
        radius = np.sqrt((min([RealDistance(turnPoint, initPoint), RealDistance(turnPoint, endPoint)])**2*(np.tan(alpha)**4-np.tan(alpha)**2*np.tan(alpha-aprox)**2))/(np.tan(alpha-aprox)**2*(1+np.tan(alpha)**2)))*(1+np.sqrt(np.tan(alpha-aprox)**2/(np.tan(alpha)**2-np.tan(alpha-aprox)**2))-np.sqrt(1+np.tan(alpha-aprox)**2/(np.tan(alpha)**2-np.tan(alpha-aprox)**2)))
        if radius < self.maxRadius:
            self.maxRadius = radius 
        radius = turnCoefficient*self.maxRadius
            
        self.b = radius/(1+np.sqrt(np.tan(alpha-aprox)**2/(np.tan(alpha)**2-np.tan(alpha-aprox)**2))-np.sqrt(1+np.tan(alpha-aprox)**2/(np.tan(alpha)**2-np.tan(alpha-aprox)**2)))
        self.a, self.m = self.b/np.tan(alpha), radius-self.b
        
        D = np.sqrt((self.b*np.sqrt(1+self.a**2*np.tan(alpha-aprox)**2/(self.b**2-self.a**2*np.tan(alpha-aprox)**2))+self.m)**2 + self.a**4*np.tan(alpha-aprox)**2/(self.b**2-self.a**2*np.tan(alpha-aprox)**2))
        if RealDistance(turnPoint, initPoint) > D:
            self.initPoint = DestinyPoint(turnPoint, D, BearingAngle(turnPoint, initPoint), attackAngle=AttackAngle(turnPoint, initPoint))
        if RealDistance(turnPoint, endPoint) > D:
            self.endPoint = DestinyPoint(turnPoint, D, BearingAngle(turnPoint, endPoint), attackAngle=AttackAngle(turnPoint, endPoint))
            
        x = self.a**2*np.tan(alpha-aprox)/np.sqrt(self.b**2-self.a**2*np.tan(alpha-aprox)**2)
        self.x = [-x]
        self.y = [self.b*np.sqrt(1+x**2/self.a**2)+self.m]
        self.distance = [0]
        self.speedDirection = []

        angle =np.arctan(np.tan(alpha-aprox))
        for i in range(1, ACCURACY+1):
            y_1 = np.tan(i*(2/ACCURACY)*angle-angle)
            self.x.append(y_1*self.a**2/np.sqrt(self.b**2-self.a**2*y_1**2))
            self.y.append(self.b*np.sqrt(1+self.x[-1]**2/self.a**2)+self.m)
            self.distance.append(self.distance[-1]+ np.sqrt((self.x[-1]-self.x[-2])**2+(self.y[-1]-self.y[-2])**2))
            self.speedDirection.append(self.GetSpeedDirection((np.arctan((self.y[-1]-self.y[-2])/(self.x[-1]-self.x[-2]))+angle)*180/np.pi))
        self.speedDirection.append(self.GetSpeedDirection(np.arctan(x*self.b /np.sqrt(self.a**4+self.a**2*x**2))*2*180/np.pi))
        
        deltaBearing, _ =IdentifyTurn(initPoint, turnPoint, endPoint, margin = 0, divide = True)
        maxHSpeed, maxVSpeed = self.drone.GetMaxSpeed(radius, deltaBearing[1])
        self.speed = min([divide(maxHSpeed, np.cos(self.minAttack*np.pi/180)), divide(maxVSpeed, np.sin(self.maxAttack*np.pi/180))])
        
        for i in range(len(self.speedDirection)-1):
            V = [np.sqrt(divide(self.drone.horizontalAccel*(self.distance[i+1]-self.distance[i]),np.sqrt((np.cos(self.speedDirection[i+1][1]*np.pi/180)*np.cos(self.speedDirection[i+1][0]*np.pi/180)-np.cos(self.speedDirection[i][1]*np.pi/180)*np.cos(self.speedDirection[i][0]*np.pi/180))**2
                                                                                                    +(np.cos(self.speedDirection[i+1][1]*np.pi/180)*np.sin(self.speedDirection[i+1][0]*np.pi/180)-np.cos(self.speedDirection[i][1]*np.pi/180)*np.sin(self.speedDirection[i][0]*np.pi/180))**2))), 
                 np.sqrt(divide(self.drone.verticalAccel*(self.distance[i+1]-self.distance[i]),np.abs(np.sin(self.speedDirection[i+1][1]*np.pi/180)-np.sin(self.speedDirection[i][1]*np.pi/180))))]
            self.speed = min(V+[self.speed])
        
        self.time = [0]
        self.accelDirection = []
        for i in range(len(self.x)-1):
            self.time.append(self.time[-1]+(self.x[i+1]-self.x[i])/self.speed)    
            self.accelDirection.append([np.sign(np.cos(self.speedDirection[i+1][1]*np.pi/180)-np.cos(self.speedDirection[i][1]*np.pi/180))*np.sqrt((np.cos(self.speedDirection[i+1][1]*np.pi/180)*np.cos(self.speedDirection[i+1][0]*np.pi/180)-np.cos(self.speedDirection[i][1]*np.pi/180)*np.cos(self.speedDirection[i][0]*np.pi/180))**2
                                               +(np.cos(self.speedDirection[i+1][1]*np.pi/180)*np.sin(self.speedDirection[i+1][0]*np.pi/180)-np.cos(self.speedDirection[i][1]*np.pi/180)*np.sin(self.speedDirection[i][0]*np.pi/180))**2)/(self.distance[i+1]-self.distance[i]),
                                        (np.sin(self.speedDirection[i+1][1]*np.pi/180)-np.sin(self.speedDirection[i][1]*np.pi/180))/(self.distance[i+1]-self.distance[i])])
                                                                    
    def GetSpeedDirection(self, alpha):
        attack = np.arcsin((np.sin(self.initAttack*np.pi/180)*np.sin((self.totalTurn-alpha)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((self.totalTurn)*np.pi/180))*180/np.pi
        self.minAttack, self.maxAttack = min([self.minAttack, np.abs(attack)]), max([self.maxAttack, np.abs(attack)])
        bearing = 180/np.pi*np.arctan2((np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((self.totalTurn-alpha)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((self.totalTurn)*np.pi/180),
                                       (np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((self.totalTurn-alpha)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((self.totalTurn)*np.pi/180))
        return [bearing, attack]
    
    def GetDistance(self):
        return self.distance[-1]
    
    def GetTime(self):
        return self.time[-1]
    
    def SetSpeed(self, newSpeed):
        if newSpeed<self.speed:
            self.speed = newSpeed 
            self.time = [0]
            for i in range(len(self.x)-1):
                self.time.append(self.time[-1]+(self.x[i+1]-self.x[i])/self.speed)   
                
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
        
        x = (self.x[i+1]-self.x[i])*(value-list[i])/(list[i+1]-list[i])+self.x[i]
        y = (self.y[i+1]-self.y[i])*(value-list[i])/(list[i+1]-list[i])+self.y[i]
        alpha = (np.arctan(x/y)+(180-self.totalTurn)*np.pi/360)*180/np.pi
        attack = np.arcsin((-np.sin(self.initAttack*np.pi/180)*np.sin((180-self.totalTurn-alpha)*np.pi/180)+np.sin(self.endAttack*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))*180/np.pi
       
        bearing = 180/np.pi*np.arctan2((-np.cos(self.initAttack*np.pi/180)*np.sin(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-alpha)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.sin(self.endBearing*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180),
                                       (-np.cos(self.initAttack*np.pi/180)*np.cos(self.initBearing*np.pi/180)*np.sin((180-self.totalTurn-alpha)*np.pi/180)+np.cos(self.endAttack*np.pi/180)*np.cos(self.endBearing*np.pi/180)*np.sin(alpha*np.pi/180))/np.sin((180-self.totalTurn)*np.pi/180))
        
        point = DestinyPoint(self.turnPoint, np.sqrt(x**2+y**2), bearing, attackAngle=attack)
        point.time = (self.time[i+1]-self.time[i])*(value-list[i])/(list[i+1]-list[i])+self.time[i]
        point.HSpeed, point.VSpeed = self.speed*np.cos(self.speedDirection[i][1]*np.pi/180), self.speed*np.sin(self.speedDirection[i][1]*np.pi/180)
        point.HAccel, point.VAccel = self.speed**2*self.accelDirection[i][0], self.speed**2*self.accelDirection[i][1]
        point.bearing = self.speedDirection[i][0]
        return point
        

class Straight: 
    def __init__(self, drone, initPoint, endPoint, initSpeed):
        self.drone = drone
        
        self.initPoint = initPoint
        self.endPoint = endPoint
        self.initSpeed = initSpeed 
        
        self.bearing, self.attack = BearingAngle(initPoint, endPoint), AttackAngle(initPoint, endPoint)
        self.maxAccel = min([divide(drone.horizontalAccel,np.cos(self.attack*np.pi/180)), np.abs(divide(drone.verticalAccel,np.sin(self.attack*np.pi/180)))])
        self.totalDistance = RealDistance(initPoint, endPoint)
    
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
        self.acceleration =  [self.maxAccel]
        
        valid = True
        
        if np.abs(endSpeed**2-self.initSpeed**2)/(2*self.maxAccel) >= self.totalDistance:
            if endSpeed < self.initSpeed:
                self.initSpeed = np.sqrt(endSpeed**2+2*self.maxAccel*self.totalDistance)
                self.speed = [self.initSpeed]
                self.acceleration = [-self.maxAccel]
                
                valid = False
            else:
                self.endSpeed = np.sqrt(self.initSpeed**2+2*self.maxAccel*self.totalDistance)

            
        else:
            maxSpeed = min([divide(self.drone.horizontalSpeed,np.cos(self.attack*np.pi/180)), np.abs(divide(self.drone.verticalSpeed,np.sin(self.attack*np.pi/180)))])
            if (maxSpeed**2-self.endSpeed**2)/(2*self.maxAccel) < (self.totalDistance-np.abs(endSpeed**2-self.initSpeed**2)/(2*self.maxAccel))/2:
                self.distance.append((maxSpeed**2-self.initSpeed**2)/(2*self.maxAccel))
                self.time.append((maxSpeed-self.speed[-1])/self.acceleration[-1])
                self.speed.append(maxSpeed)
                self.acceleration.append(0)
                if maxSpeed == self.endSpeed:
                    self.distance.append(self.totalDistance)
                    self.time.append(self.time[-1]-(self.distance[-1]-self.distance[-2])/maxSpeed)
                    self.speed.append(maxSpeed)
                    self.acceleration.append(0)
                    return valid
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
        self.time.append(self.time[-1]+(self.endSpeed-self.speed[-1])/self.acceleration[-1])
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
        
        return point
        
        
                
        
        
    
    
    
            
            
            