from Library.LatLonProjection import *
from Library.Discretization import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import imageio
import os, io
import shutil
from tslearn.metrics import dtw_path

import math
FPS = 3


class Graph:
    def __init__(self, pointList, type, *, label =None , color= None, marker = None, opacity = 1):
        self.pointList = pointList
        self.type = type #Scatter[S], Plot[P], Quiver[Q]
        self.label = label
        self.color = color
        self.marker = marker
        self.opacity = opacity


def PointsToDataSet(pointList, projection = "LL",*, origin = None): #magnitude--> LatLong [LL]  || Meters [M]
    if origin == None:
        origin = pointList[0]  
    dataSet, speedVector = [[],[],[]],[[],[],[]]
    for point in pointList:
        if projection == "LL":
            dataSet[0].append(point.lat)
            dataSet[1].append(point.lon)
            speed = DestinyPoint(point, point.HSpeed, point.bearing, deltaAltitude=point.VSpeed)
            speedVector[0].append(speed.lat-point.lat)
            speedVector[1].append(speed.lon-point.lon)
            speedVector[2].append(speed.alt-point.alt)
            
        elif projection == "M":
            distance = LatLonDistance(origin, point)
            bearing = BearingAngle(origin, point)
            dataSet[0].append(distance*math.cos(bearing/180*math.pi))
            dataSet[1].append(distance*math.sin(bearing/180*math.pi))
            speedVector[0].append(point.HSpeed*math.cos(point.bearing*np.pi/180))
            speedVector[1].append(point.HSpeed*math.sin(point.bearing*np.pi/180))
            speedVector[2].append(point.VSpeed)
            
        dataSet[2].append(point.alt)
    return dataSet, speedVector

def PointsToList(pointList, projection = "LL", *, origin = None):
    DataSet, _ = PointsToDataSet(pointList, projection, origin = origin)
    list = []
    for i in range(len(DataSet[0])):
        point =  []
        point.append(DataSet[0][i])
        point.append(DataSet[1][i])
        point.append(DataSet[2][i])
        list.append(point)
    return list

def Scatter3D(dataSet, label, color, marker = None, opacity = 1):
    ax.scatter(dataSet[0], dataSet[1], dataSet[2], label = label,c= color, marker = marker, alpha = opacity)

def Plot3D(dataSet, label, color, marker = None, opacity = 1):
    ax.plot(dataSet[0], dataSet[1], dataSet[2], label = label,c= color, marker = marker, alpha = opacity)
    

def Quiver3D(dataSet, speedVector, magnitude):
    norm = plt.Normalize(min(magnitude), max(magnitude))
    colors = cm.plasma(norm(magnitude))
    for i in range(len(dataSet[0])):
        ax.quiver(dataSet[0][i], dataSet[1][i], dataSet[2][i],
                speedVector[0][i], speedVector[1][i], speedVector[2][i],
                color=colors[i], length=5, normalize=True)
    mappable = cm.ScalarMappable(norm=norm, cmap='plasma')
    mappable.set_array(magnitude)
    fig.colorbar(mappable, shrink=0.5, aspect=5)
    # cbar = plt.colorbar(strm, ax=ax)
    # cbar.set_label('Speed (m/s)')
    

def RecordDiscretization(discretization, title,*,pointList = None, recordSpeed=1, zoom = 1):
    flightPlan = []
    for trace in discretization.flightPlan["traces"]:
        flightPlan= flightPlan+trace
    FlightPlan_DataSet, _ = PointsToDataSet(flightPlan, "M", origin=flightPlan[0]) 
    mainPoints = discretization.GetPoints()
    discPoints = discretization.GetPoints(time = recordSpeed/FPS)
    Discretize_DataSet, speedVector = PointsToDataSet(discPoints, "M", origin=flightPlan[0])
    dataSet = [FlightPlan_DataSet[0]+Discretize_DataSet[0],FlightPlan_DataSet[1]+Discretize_DataSet[1],FlightPlan_DataSet[2]+Discretize_DataSet[2]]
    
    if pointList != None:
        pointList_DataSet, _ = PointsToDataSet(pointList, "M" , origin=flightPlan[0]) 
        path, dtw_distance = dtw_path(PointsToList(pointList, "M", origin=flightPlan[0]), PointsToList(discPoints,"M", origin=flightPlan[0]))
        links = [[i, j] for (i, j) in path] 
        dataSet[0], dataSet[1], dataSet[2] = dataSet[0]+pointList_DataSet[0], dataSet[1]+pointList_DataSet[1], dataSet[2]+pointList_DataSet[2] 
    
    axisTicks =[[min(dataSet[0]),max(dataSet[0])],[min(dataSet[1]),max(dataSet[1])], [min(dataSet[2]),max(dataSet[2])]]

    
    norm = plt.Normalize(0, np.sqrt(discretization.drone.horizontalSpeed**2+discretization.drone.verticalSpeed**2))
    images = []
    output_dir = "VideoPictures"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    else:
        for filename in os.listdir(output_dir):
            file_path = os.path.join(output_dir, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)  # Remove the file or link
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)  # Remove the directory
            except Exception as e:
                print(f'Failed to delete {file_path}. Reason: {e}')
    
    m, n = 2, 0 
    if m >=len(mainPoints): m = 1
    
    time = []
    HSpeed, VSpeed = [], []
    
    for i in range(len(discPoints)):
        print(i)
        time.append(discPoints[i].time)
        HSpeed.append(discPoints[i].HSpeed), VSpeed.append(discPoints[i].VSpeed) 
        
        #1. Set Figure Configuration
        fig = plt.figure(constrained_layout=True,figsize=(14, 7))
        gs = fig.add_gridspec(2, 40)
        #2. Graph Distribution
        ax1 = fig.add_subplot(gs[:, 0:26], projection='3d')
        ax2 = fig.add_subplot(gs[0, 26:39])
        ax2_B = fig.add_subplot(gs[0, 39])
        ax3 = fig.add_subplot(gs[1, 26:39])
        ax3_B = fig.add_subplot(gs[1, 39])
        
        #3. Set Axes
        ax1.set_title(f"{title} -- {round(discPoints[i].time,2)}s")
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.set_zlabel('Altitude [m]')
        
        ax2.set_title(f'HORIZONTAL COMPONENT')
        ax2.set_ylabel("Speed [m/s]")
        ax2.set_ylim(0-0.5, discretization.drone.horizontalSpeed+.5)
        ax2_B.set_yticks([])
        ax2_B.set_yticklabels([])
        ax2_B = ax2_B.twinx()
        ax2_B.set_ylabel("Accel [m/s^2]")
        ax2_B.set_xticks([])
        ax2_B.set_xticklabels([])
        color = "black"
        if discPoints[i].HAccel > 0: 
            color = "green"
            ax2_B.set_ylim(0, discretization.drone.horizontalAccel)
            ax2_B.set_yticks([0, discretization.drone.horizontalAccel])
        elif discPoints[i].HAccel < 0:
            color = "red"
            ax2_B.set_ylim(-discretization.drone.horizontalAccel, 0)
            ax2_B.set_yticks([-discretization.drone.horizontalAccel, 0])
        else:
            ax2_B.set_ylim(0, discretization.drone.verticalAccel)
        
        ax3.set_title(f'VERTICAL COMPONENT')
        ax3.set_ylabel("Speed [m/s]")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylim(-discretization.drone.verticalSpeed-.5, discretization.drone.verticalSpeed+.5)
        ax3_B.set_yticks([])
        ax3_B.set_yticklabels([])
        ax3_B = ax3_B.twinx()
        ax3_B.set_ylabel("Accel [m/s^2]")
        ax3_B.set_xticks([])
        ax3_B.set_xticklabels([])
        
        
        
        #4. Insert Data
        
        if np.sqrt(discPoints[i].HSpeed**2+discPoints[i].VSpeed**2) !=0:
            ax1.quiver(Discretize_DataSet[0][i], Discretize_DataSet[1][i], Discretize_DataSet[2][i], speedVector[0][i], speedVector[1][i], speedVector[2][i], color=cm.plasma(norm(np.sqrt(discPoints[i].HSpeed**2+discPoints[i].VSpeed**2))), length=10, normalize=True)
        ax1.scatter(Discretize_DataSet[0][i], Discretize_DataSet[1][i], Discretize_DataSet[2][i], color=cm.plasma(norm(np.sqrt(discPoints[i].HSpeed**2+discPoints[i].VSpeed**2))))
        if (i-1)>=0:
            ax1.plot(Discretize_DataSet[0][i-1:i+1], Discretize_DataSet[1][i-1:i+1], Discretize_DataSet[2][i-1:i+1], color=cm.plasma(norm(np.sqrt(discPoints[i-1].HSpeed**2+discPoints[i-1].VSpeed**2))), linestyle = '--')
            if (i-2)>=0:
                ax1.plot(Discretize_DataSet[0][0:i], Discretize_DataSet[1][0:i], Discretize_DataSet[2][0:i], color="black", linestyle = '-.')
        while time[-1] > mainPoints[m].time:
            m+=2
            if m >= len(mainPoints): 
                m = len(mainPoints)-1
                break 
        
        if m < len(mainPoints)-1:
            ax1.scatter(FlightPlan_DataSet[0][int(m/2):int(m/2+2)], FlightPlan_DataSet[1][int(m/2):int(m/2+2)], FlightPlan_DataSet[2][int(m/2):int(m/2+2)], marker = "o",color = "red") 
            ax1.scatter(FlightPlan_DataSet[0][int(m/2+2):], FlightPlan_DataSet[1][int(m/2+2):], FlightPlan_DataSet[2][int(m/2+2):])
            ax1.scatter(FlightPlan_DataSet[0][0:int(m/2)], FlightPlan_DataSet[1][0:int(m/2)], FlightPlan_DataSet[2][0:int(m/2)], marker= "X")
        else:
            ax1.scatter(FlightPlan_DataSet[0][-1], FlightPlan_DataSet[1][-1], FlightPlan_DataSet[2][-1], marker = "*",color = "red")
            ax1.scatter(FlightPlan_DataSet[0][int((m+1)/2+2):], FlightPlan_DataSet[1][int((m+1)/2+2):], FlightPlan_DataSet[2][int((m+1)/2+2):])
            ax1.scatter(FlightPlan_DataSet[0][0:int((m+1)/2)], FlightPlan_DataSet[1][0:int((m+1)/2)], FlightPlan_DataSet[2][0:int((m+1)/2)], marker= "X")
        
        if pointList != None:
            ax1.plot(pointList_DataSet[0], pointList_DataSet[1], pointList_DataSet[2], label = "Telemetry",c= "blue", linestyle = "-.", alpha = 0.5)
            
            
            # for h, k in links[0:n]:    
            #     ax1.plot([pointList_DataSet[0][h], Discretize_DataSet[0][k]], [pointList_DataSet[1][h], Discretize_DataSet[1][k]], [pointList_DataSet[2][h], Discretize_DataSet[2][k]], color = "yellow",linewidth=.7)
            
            for h, k in links[n:]:
                if k != i:
                    break
                ax1.plot([pointList_DataSet[0][h], Discretize_DataSet[0][k]], [pointList_DataSet[1][h], Discretize_DataSet[1][k]], [pointList_DataSet[2][h], Discretize_DataSet[2][k]], label = "Related Points", color = "yellow",linewidth=3)
                n+=1
            
            
        if zoom != 1:
            ax1.set_xlim(Discretize_DataSet[0][i]-(axisTicks[0][1]-axisTicks[0][0])/zoom, Discretize_DataSet[0][i]+(axisTicks[0][1]-axisTicks[0][0])/zoom)
            ax1.set_ylim(Discretize_DataSet[1][i]-(axisTicks[1][1]-axisTicks[1][0])/zoom, Discretize_DataSet[1][i]+(axisTicks[1][1]-axisTicks[1][0])/zoom)

        
        ax2.plot(time, HSpeed, color = "black", linewidth= 3)
        ax2.fill_between(time, HSpeed, color = "blue", alpha = .7)
        color = "black"
        if discPoints[i].HAccel > 0: 
            color = "green"
            ax2_B.set_ylim(0, discretization.drone.horizontalAccel)
            ax2_B.set_yticks([0, discretization.drone.horizontalAccel])
        elif discPoints[i].HAccel < 0:
            color = "red"
            ax2_B.set_ylim(-discretization.drone.horizontalAccel, 0)
            ax2_B.set_yticks([-discretization.drone.horizontalAccel, 0])
        else:
            ax2_B.set_ylim(0, discretization.drone.horizontalAccel)
        ax2_B.bar(1, discPoints[i].HAccel, color=color, alpha=0.7)
        ax2.scatter([time[-1]], [HSpeed[-1]], color = color, linewidths= 3)

        ax3.plot(time, VSpeed, color = "black", linewidth= 4)
        ax3.fill_between(time, VSpeed, color = "blue", alpha = .7)
        color = "black"
        if discPoints[i].VAccel > 0: 
            color = "green"
            ax3_B.set_ylim(0, discretization.drone.verticalAccel)
            ax3_B.set_yticks([0, discretization.drone.verticalAccel])
        elif discPoints[i].VAccel < 0:
            color = "red"
            ax3_B.set_ylim(-discretization.drone.verticalAccel, 0)
            ax3_B.set_yticks([-discretization.drone.verticalAccel, 0])
        else:
            ax3_B.set_ylim(0, discretization.drone.verticalAccel)
        ax3_B.bar(1, discPoints[i].VAccel, color=color, alpha=0.7)
        ax3.scatter([time[-1]], [VSpeed[-1]], color = color, linewidths= 3)
       
        
        '''
         plt.tight_layout()
        
        filename = f"{output_dir}/frame_{i}.png"
        plt.savefig(filename)
        images.append(imageio.v3.imread(filename))
            plt.close()
          plt.clf()
        
        '''
        #5. Recording
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        images.append(imageio.v3.imread(buf))
        buf.close()
        plt.close()
        plt.clf()
    
    imageio.mimsave('simulation.mp4', images, fps=FPS)
        
def Show3DGraph(graphList, title, projection): #magnitude--> LatLong [LL]  || Meters [M]
    global fig, ax
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    fig.suptitle(title, fontsize=16)
    
    
    if projection == "LL": xLabel, yLabel = "Latitude", "Longitude"
    elif projection =="M": xLabel, yLabel = "x [m]", "y [m]"
    ax.set_xlabel(xLabel)
    ax.set_ylabel(yLabel)
    ax.set_zlabel('Altitude (m)')

    dataSet, _ = PointsToDataSet(graphList[0].pointList)
    axisTicks =[[dataSet[0][0],dataSet[0][0]],[dataSet[1][0],dataSet[1][0]], [dataSet[2][0],dataSet[2][0]]]
    for graph in graphList:
        dataSet, _ = PointsToDataSet(graph.pointList)
        axisTicks[0][0]= min(dataSet[0]+[axisTicks[0][0]])
        axisTicks[0][1]= max(dataSet[0]+[axisTicks[0][1]])
        axisTicks[1][0]= min(dataSet[1]+[axisTicks[1][0]])
        axisTicks[1][1]= max(dataSet[1]+[axisTicks[1][1]])
        axisTicks[2][0]= min(dataSet[2]+[axisTicks[2][0]])
        axisTicks[2][1]= max(dataSet[2]+[axisTicks[2][1]])


    for graph in graphList:
        dataSet, speedVector = PointsToDataSet(graph.pointList, projection, graphList[0].pointList[0])
        match graph.type:
            case "S":
                Scatter3D(dataSet, graph.label, graph.color, graph.marker)
            case "P":
                Plot3D(dataSet, graph.label, graph.color, graph.marker)
            case "Q":
                magnitude = []
                for point in graph.pointList:
                    magnitude.append(np.sqrt(point.HSpeed**2+point.VSpeed**2))
                Quiver3D(dataSet, speedVector, magnitude)
    # if projection == "LL":
    #     axisTicks =[[round(axisTicks[0][0], 4),round(axisTicks[0][1], 4)],[round(axisTicks[1][0], 4),round(axisTicks[1][1], 4)]]
    #     ax.set_xticks(axisTicks[0], [str(axisTicks[0][0]), str(axisTicks[0][1])])
    #     ax.set_yticks(axisTicks[1], [str(axisTicks[1][0]), str(axisTicks[1][1])])
    

          
    
    ax.legend()
    plt.show()


def CompareResults(discretization, pointList, title):
    discPoints = discretization.GetPoints(time = 0.1)
    path, dtw_distance = dtw_path(PointsToList(pointList, "M", origin=pointList[0]), PointsToList(discPoints,"M", origin=pointList[0]))
    links = [[i, j] for (i, j) in path]
    meanDeviation = 0
    time, x = [0], 0
    pointType = ["Straight"] 
    count = 1
    for i, j in links:
        if j > 12:
            if x != i:
                time[-1] = time[-1]/count   
                x, count = x+1, 1
                pointType.append(discPoints[j].FPLvel)
                time.append(0)
            else:
                count+=1
            time[-1]+=  discPoints[j].time-pointList[i].time
        else:
            x = i+1
    
            
    # print([t-meanDeviation for t in time])
    
    i = 0
    turn, straight, hover = [], [], []
    while i < len(pointType):
        if pointType[i] == "Turn":
            turn.append(i)
        elif pointType[i] == "Straight":
            straight.append(i)
        else:
            while pointType[i] == "Hover":
                hover.append(i)
                i+=1
            meanDeviation = sum(time[min([straight[0], turn[0]]):min([straight[0], turn[0]])+int((hover[0]-1-min([straight[0], turn[0]]))/20)])/+int((hover[0]-1-min([straight[0], turn[0]]))/20)
            plt.bar(straight, [time[s]-meanDeviation for s in straight], color = "green", label = "Straights")
            plt.bar(turn, [time[t]-meanDeviation for t in turn], color = "blue", label = "Turns")
            plt.plot(hover, [0 for h in hover], color = "red", label = "Hover")
            turn, straight, hover = [], [], []
        i+=1
    meanDeviation = sum(time[min([straight[0], turn[0]]):min([straight[0], turn[0]])+int((len(time)-min([straight[0], turn[0]]))/20)])/int((len(time)-1-min([straight[0], turn[0]]))/20)
    plt.bar(straight, [time[s]-meanDeviation for s in straight], color = "green", label = "Straights")
    plt.bar(turn, [time[t]-meanDeviation for t in turn], color = "blue", label = "Turns")
    plt.plot(hover, [0 for h in hover], color = "red", label = "Hover")
    plt.xlabel("Real Point")
    plt.ylabel("Time Deviation [s]")
    plt.title(title)
    plt.legend()
    plt.show()
    



