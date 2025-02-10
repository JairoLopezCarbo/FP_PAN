from Library.FilesReader import *
from Library.Graphing import *
from Library.LatLonProjection import *
from Library.Discretization import *
from Library.Drone import *
from Library.Analysis import *
import matplotlib.pyplot as plt
import random as rnd
import json




def ShowFPandTelem(flightName):
    
    flightPlan = CsvPointListReader(f"Data/FlightPlan/FlightPlan_{flightName}.csv") 
    pointList = CsvPointListReader(f"Data/Telemetry/Telemetry_{flightName}.csv") 
    origin = flightPlan.pop(0)
    destination = flightPlan.pop(len(flightPlan)-1)
    
    Orig_Graph = Graph([origin], "S", "Start", "red", "*")
    Dest_Graph = Graph([destination], "S", "End", "red", "X")
    FP_Graph = Graph(flightPlan, "S", "FlightPlan", "black", None, 0.0)
    Telem_Graph = Graph(pointList, "P", "Telemetry", "black", None, 0.0)
    
    # turnPoints, turnIndex, straightPoints, straightIndex, flightPlan = TelemetryTo_CurvesAndStraights(pointList, flightPlan)
    # turns = []
    # for turn in turnPoints:
    #     turns = turns + turn
    # straights = []
    # for straight in straightPoints:
    #     straights = straights + straight
    
    # Turn_Graph = Graph(turns, "S", "Turns", "purple")
    # Straight_Graph = Graph(straights, "S", "Straights", "green")

    Show3DGraph([Orig_Graph, Dest_Graph, FP_Graph, Telem_Graph], flightName, "M")



droneData = CsvAvailableDataReader("Data/flight_to_drone.csv")

for drone in droneData.keys():    
    for flightName in droneData[drone]:
        flightPlan = CsvPointListReader(f"Data/FlightPlan/FlightPlan_{flightName}.csv")
        pointList = CsvPointListReader(f"Data/Telemetry/Telemetry_{flightName}.csv") 
        
        drone = Drone(12, 6, 2, 1)
        discretize = Discretize(flightPlan, drone)
        
        totalTurns  = 0
        for trace in discretize.flightPlan["traces"]:
            totalTurns+= len(trace)-2
            
        discretize.SetTurns([1 for _ in range(totalTurns)])
        
        with open("points.json", "w") as json_file:
            json.dump([point.to_dict() for point in discretize.GetPoints(time = 0.2)], json_file, indent=4)
        # d = DroneProfile(flightPlan, pointList)
        # print(d)
        CompareResults(discretize, pointList, flightName)
        # input("S")
        # RecordDiscretization(discretize, flightName, pointList = pointList, zoom = 7, recordSpeed = 2)
        
        
        
        # # ShowFPandTelem(flightName)
        # # flightPlan = flightPlan[2:5]
        # drone = Drone(15, 10, 5, 3)
        # discretize = Discretize(flightPlan, drone)
        # turnCoeficcientList = [.5 for _ in range(len(discretize.flightPlan)-2)]
        # discretize.SetTurns(turnCoeficcientList)
        
        # mainPoints = discretize.GetPoints()
        # discretizedPoints = discretize.GetPoints(time = 1)
        # Show3DGraph([Graph(flightPlan, "S", label = "FlightPlan", color = "red"),Graph(mainPoints, "S", label = "mainPoints", color = "black"), Graph(discretizedPoints, "Q", label = "Discretized")], "flightName", "LL")
        # fig, axs = plt.subplots(4, 1, figsize=(10, 20))
        # time, HSpeed, VSpeed, HAccel, VAccel = [],[],[],[],[]
        # for point in points2:
        #     time.append(point.time)
        #     HSpeed.append(point.HSpeed)
        #     VSpeed.append(point.VSpeed)
        #     HAccel.append(point.HAccel)
        #     VAccel.append(point.VAccel)
        # y = [HSpeed, VSpeed, HAccel, VAccel]
        # name = ["HSpeed", "VSpeed", "HAccel", "VAccel"]
        # for i, ax in enumerate(axs):
        #     ax.plot(time, y[i])
        #     ax.grid(True)
        #     ax.set_xlabel('Time')
        #     ax.set_ylabel(name[i])
        # plt.show()
            
        
        

    
