from Library.LatLonProjection import *
import numpy as np
from tslearn.metrics import dtw_path
import Library.Drone as Drone
from Library.Discretization import *  
from Library.Graphing import *  

FORWARD_POS = 5
def FindLatLonMargins(pointList):
    """Finds the LatLongAlt margins of a list of points

    Args:
        pointList (List<Point>): 

    Returns:
        List<double[2]>: [[minLat, maxLat], [minLong, maxLong], [minAlt, maxAlt]]
    """
    margins = [[pointList[0].lat,pointList[0].lat],[pointList[0].lon,pointList[0].lon], [pointList[0].alt,pointList[0].alt]]
    for point in pointList:
        if point.lat < margins[0][0]: margins[0][0] = point.lat
        if point.lat > margins[0][1]: margins[0][1] = point.lat
        if point.lon < margins[1][0]: margins[1][0] = point.lon
        if point.lon > margins[1][1]: margins[1][1] = point.lon
        if point.alt < margins[2][0]: margins[2][0] = point.alt
        if point.alt > margins[2][1]: margins[2][1] = point.alt
    return margins

def ConvertToIndexList(searchList, targetList):
    """Matches the minorList with the pointList and returns 
    the position in the List of each point

    Args:
        searchList (List<Point>): Points to search in targetList
        targetList (List<Point>): It contains all points in searchList

    Returns:
        List<int>: List of indexes of the points of searchList in targetList
    """
    indexList = []
    i = 0
    for point in searchList:
        while not(targetList[i].lat==point.lat and targetList[i].lon==point.lon and targetList[i].alt==point.alt):
            i += 1
        indexList.append(i)
        i +=1
    return indexList

def Speed_Acceleration(pointList, timeMargin, ):
    speedList = []
    accelList = []
    for i in range(len(pointList)):
        speed, deltaTime = 0, 0
        aprox , j = 0, 1
        while deltaTime < timeMargin:
            if i+j < len(pointList) and pointList[i].time!=pointList[i+j].time:
                speed += (1/j)*RealDistance(pointList[i], pointList[i+j])/np.abs(pointList[i].time-pointList[i+j].time)
                deltaTime += np.abs(pointList[i].time-pointList[i+j].time)
                aprox += 1/j
            if i-j >= 0 and pointList[i].time!=pointList[i-j].time:
                speed += (1/j)*RealDistance(pointList[i], pointList[i-j])/np.abs(pointList[i].time-pointList[i-j].time)
                deltaTime += np.abs(pointList[i].time-pointList[i-j].time)
                aprox += 1/j
            j +=1
        speedList.append(speed/aprox)
        if i!= 0 and pointList[i].time!=pointList[i-1].time:
            accelList.append((speedList[i]-speedList[i-1])/(pointList[i].time-pointList[i-1].time))
        elif i!=0 and pointList[i].time==pointList[i-1].time:
            accelList.append(accelList[-1])
        else:
            accelList.append(0)
            
        
    accelList.append(0)
    
    return speedList, accelList

def RDP_algorithm(pointList, maxDistance):#, return a PointList
    """Ramer-Douglas-Peucker (RDP) algorithm to simplify the path 

    Args:
        pointList (List<Point>): path with too many points 
        maxDistance (double): max distance of dispersion

    Returns:
        List<Point>: Simplified path
        List<int>: Index of the simplification points in the input list
    """

    index = -1
    maxDistanceFound = maxDistance
    
    for i in range(1, len(pointList)-1):
        if DistanceFromLineProjection([pointList[0],pointList[-1]], pointList[i]) > maxDistanceFound:
            index = i
            maxDistanceFound = DistanceFromLineProjection([pointList[0],pointList[-1]], pointList[i])
    if index != -1:
        first_half, _ = RDP_algorithm(pointList[:index+1], maxDistance)
        second_half, _ = RDP_algorithm(pointList[index:], maxDistance)
        second_half.pop(0)
        return first_half+second_half, ConvertToIndexList(first_half+second_half, pointList)
    else:
        return [pointList[0], pointList[-1]], ConvertToIndexList([pointList[0], pointList[-1]], pointList)


def TurnGathering(pointList, simpPointsIndex):
    """With a pointList and the position of the simplificated path, 
    it detects whats a turn all along the route.

    Args:
        pointList (List<Point>): Route Discretized Points
        simpPointsIndex (List<int>): SimplifiedPath index list using RDP_algorithm

    Returns:
        List<List<Point>>: List of turns, each turn is a List<Point>
        List<List<int>>: List of turns, each turn is a List<int> wich are the reference positon in pointList
    """
    turnList = []
    averageDistance = 0
    for i in range(len(simpPointsIndex)-1):
        averageDistance += RealDistance(pointList[simpPointsIndex[i]], pointList[simpPointsIndex[i+1]])
    averageDistance/= len(simpPointsIndex)-1
    
    lastTurn = "None"    
    for i in range(1, len(simpPointsIndex)-1):
        prevPoint, currentPoint, forwPoint = pointList[simpPointsIndex[i-1]], pointList[simpPointsIndex[i]], pointList[simpPointsIndex[i+1]]
        turn, angle = IdentifyTurn(prevPoint, currentPoint, forwPoint, margin = 0.3)
        if turn == "None":
            lastTurn = turn
        elif turn != lastTurn and np.abs(angle) > .5:
            lastTurn = turn 
            turnList.append([simpPointsIndex[i]])
        else:
            if RealDistance(currentPoint, pointList[turnList[-1][-1]]) > averageDistance*2:
                turnList.append([simpPointsIndex[i]])
            else:
                turnList[-1].append(simpPointsIndex[i])
   
   
    turnPoints, turnIndex = [], []
    for turn in turnList:
        if len(turn) > 3:
            turnPoints.append([])
            turnIndex.append([])
            init, fin = pointList[turn[0]], pointList[turn[1]]            
            while RealDistance(init, pointList[turn[0]])<.2:
                turn[0]-=1
            while RealDistance(init, pointList[turn[-1]])<.2:
                turn[-1]+=1
                
            for pos in range(turn[0], turn[-1]+1):
                turnPoints[-1].append(pointList[pos])
                turnIndex[-1].append(pos)
                
    
    return turnPoints, turnIndex
    
     
def TurnsWithFlightPlan(flightPlan, pointList, turnIndex):
    """Given a flightPlan and the pointList index position of some turns 
    it improves the found turns and it joins the ones which are around the same flightPlan point

    Args:
        flightPlan (List<Point>): 
        pointList (List<Point>): discretized FlightPlan
        turnIndex (List<List<int>): List of turns with the reference position in pointList

    Returns:
        List<List<Point>>: The List of FlightPlan points that reference each turn
        List<List<Point>>: List of turns, each turn is a List<Point>
        List<List<int>>: List of turns, each turn is a List<int> wich are the reference positon in pointList
    """
    turns = []
    flightPlanPoints = []
    for turn in turnIndex:
        minDistance = np.Infinity
        nearestFLPoint = None
        startPoint, midPoint, endPoint = pointList[turn[0]], pointList[turn[int(len(turn)/2-1)]], pointList[turn[-1]]
        for FPPoint in flightPlan:
            distance = RealDistance(FPPoint, startPoint) + RealDistance(FPPoint, midPoint) +RealDistance(FPPoint, endPoint)
            if distance<minDistance:
                minDistance = distance
                nearestFLPoint = FPPoint

        if nearestFLPoint in flightPlanPoints: 
            turns[flightPlanPoints.index(nearestFLPoint)]
        else:
            turns.append(turn)
            flightPlanPoints.append(nearestFLPoint)
    
    for i in range(len(turns)):
        turns[i].sort()
        if RealDistance(pointList[turns[i][0]], flightPlanPoints[i]) > RealDistance(pointList[turns[i][-1]], flightPlanPoints[i]):
            maxdistance = RealDistance(pointList[turns[i][0]], flightPlanPoints[i])
            while turns[i][-1] <len(pointList) and maxdistance > RealDistance(pointList[turns[i][-1]], flightPlanPoints[i]):
                turns[i][-1] +=1
        else:
            maxdistance = RealDistance(pointList[turns[i][-1]], flightPlanPoints[i])
            while turns[i][0] > 0 and maxdistance > RealDistance(pointList[turns[i][0]], flightPlanPoints[i]):
                turns[i][0] -=1
    
    turnPoints, turnIndex = [], []
    for turn in turns:
        turnPoints.append([])
        turnIndex.append([])
        for pos in range(turn[0], turn[-1]):
            turnPoints[-1].append(pointList[pos])
            turnIndex[-1].append(pos)
    return flightPlanPoints, turnPoints, turnIndex


def TelemetryTo_CurvesAndStraights (pointList, flightPlan = None):
    _ , simpPointsIndex = RDP_algorithm(pointList, .1) 
    _ , turnIndex = TurnGathering(pointList, simpPointsIndex)
    if flightPlan != None:
        flightPlan, turnPoints, turnIndex = TurnsWithFlightPlan(flightPlan, pointList, turnIndex)
    else:
        turnPoints = []
        for turn in turnIndex:
            turnPoints.append([])
            for index in turn:
                turnPoints[-1].append(pointList[index])
    straightPoints = []
    straightIndex = []
    for straight in range(len(turnIndex)-1):
        initPos, endPos = turnIndex[straight][-1]+1, turnIndex[straight+1][0]
        if np.abs(IdentifyTurn(pointList[initPos], pointList[int((endPos-initPos)/2+ initPos)], pointList[endPos], margin = 0)[1]) > 2:
            continue
        straightPoints.append([])
        straightIndex.append([])
        for index in range(turnIndex[straight][-1]+1, turnIndex[straight+1][0]):
            straightPoints[-1].append(pointList[index])
            straightIndex[-1].append(index)
    return turnPoints, turnIndex, straightPoints, straightIndex, flightPlan

def DroneProfile(flightPlan, pointList):
    drone = Drone(14, 6, 2, 1)
    discretization = Discretize(flightPlan, drone)  
    totalTurns  = 0
    for trace in discretization.flightPlan["traces"]:
        totalTurns+= len(trace)-2
            
    discretization.SetTurns([1 for _ in range(totalTurns)])
    discPoints = discretization.GetPoints(time = 0.1)
    path, dtw_distance = dtw_path(PointsToList(pointList, "M", origin=pointList[0]), PointsToList(discPoints,"M", origin=pointList[0]))
    links = [[i, j] for (i, j) in path]
    
    x = 0
    points, pointType =  [], ["Straight"]
    count = 1
    for i, j in links:
        if j > 12:
            if x != i:  
                x +=1 
                points.append(pointList[i])
                pointType.append(discPoints[j].FPLvel)
                
            else:
                count+=1
        else:
            x = i+1
    
            
    # print([t-meanDeviation for t in time])
    
    i = 0
    turns,straight = {"speed": [], "radius": [], "deltaBearing": [], "deltaAttack": []}, {"h_speed": 0, "v_speed": 0, "h_accel": [], "v_accel": []}
    for _ in discretization.traces:
        for trace in _:
            type = trace.type
            a = i
            while (pointType[a] == type or pointType[a]=="Hover") and a < len(pointType)-1:
                a +=1
                if pointType[a] == "Hover":
                    i+=1
            
            if type == "Turn":
                turns["speed"].append(trace.totalDistance/(pointList[a].time-pointList[i].time))
                turns["radius"].append(trace.R)
                turns["deltaBearing"].append(trace.deltaBearing[1])
                turns["deltaAttack"].append(trace.deltaAttack[1])
            elif type == "Straight":
                prevSpeed = 0
                for p in range (i, a-1):
                    speed = RealDistance(pointList[p], pointList[p+1])/(pointList[p+1].time-pointList[p].time)
                    straight["h_speed"] = max([straight["h_speed"],speed*np.abs(np.cos(trace.bearing*np.pi/180))])
                    straight["v_speed"] = max([straight["v_speed"],speed*np.sin(trace.attack*np.pi/180)])
                    if prevSpeed != 0:
                        if np.abs(speed-prevSpeed)/(pointList[p].time-pointList[p-1].time)*np.abs(np.cos(trace.bearing*np.pi/180)) > 0.1:
                            straight["h_accel"].append(np.abs(speed-prevSpeed)/(pointList[p].time-pointList[p-1].time)*np.abs(np.cos(trace.bearing*np.pi/180)))
                        if np.abs((speed-prevSpeed)/(pointList[p].time-pointList[p-1].time)*np.sin(trace.attack*np.pi/180)) > 0.1:
                            straight["v_accel"].append(np.abs((speed-prevSpeed)/(pointList[p].time-pointList[p-1].time)*np.sin(trace.attack*np.pi/180)))
                    prevSpeed = speed
            i=a
    straight["h_accel"] = np.mean(straight["h_accel"])
    straight["v_accel"] = np.mean(straight["v_accel"])
    return turns, straight
    