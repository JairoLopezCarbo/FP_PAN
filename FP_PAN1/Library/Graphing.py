from Library.LatLonProjection import *
from Library.Discretization import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import imageio
import os

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


def PointsToDataSet(pointList, projection = "LL", origin = None): #magnitude--> LatLong [LL]  || Meters [M]
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

def RecordDiscretization(discretization, title,*,recordSpeed=1):
    
    
    flightPlan = discretization.flightPlan
    DS_FP, _ = PointsToDataSet(flightPlan, "M", flightPlan[0]) 
    mainPoints = discretization.GetPoints()
    discPoints = discretization.GetPoints(time = recordSpeed/FPS)
    DISC_FP, speedVector = PointsToDataSet(discPoints, "M", flightPlan[0]) 
    
    norm = plt.Normalize(0, np.sqrt(discretization.drone.horizontalSpeed**2+discretization.drone.verticalSpeed**2))
    images = []
    output_dir = "VideoPictures"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    m = 2 
    if m >=len(mainPoints): m = 1
    
    time = []
    HSpeed, VSpeed = [], []
    for i in range(len(discPoints)):
        fig = plt.figure(figsize=(14, 7))
        gs = fig.add_gridspec(2, 2, width_ratios=[2, 1])
        ax1 = fig.add_subplot(gs[:, 0], projection='3d')
        ax1.set_title(f"{title} -- {round(discPoints[i].time,2)}s")
        ax1.quiver(DISC_FP[0][i], DISC_FP[1][i], DISC_FP[2][i], speedVector[0][i], speedVector[1][i], speedVector[2][i], color=cm.plasma(norm(np.sqrt(discPoints[i].HSpeed**2+discPoints[i].VSpeed**2))), length=10, normalize=True)
        ax1.scatter(DISC_FP[0][i], DISC_FP[1][i], DISC_FP[2][i], color=cm.plasma(norm(np.sqrt(discPoints[i].HSpeed**2+discPoints[i].VSpeed**2))))
        if (i-1)>=0:
            ax1.plot(DISC_FP[0][i-1:i+1], DISC_FP[1][i-1:i+1], DISC_FP[2][i-1:i+1], color=cm.plasma(norm(np.sqrt(discPoints[i-1].HSpeed**2+discPoints[i-1].VSpeed**2))), linestyle = '--')
            if (i-2)>=0:
                ax1.plot(DISC_FP[0][0:i], DISC_FP[1][0:i], DISC_FP[2][0:i], color="black", linestyle = '-.')
        
        
        time.append(discPoints[i].time)
        while time[-1] > mainPoints[m].time:
            m+=2
            if m >= len(mainPoints): 
                m = len(mainPoints)-1
                break 
        
        if m < len(mainPoints)-1:
            ax1.scatter(DS_FP[0][int(m/2):int(m/2+2)], DS_FP[1][int(m/2):int(m/2+2)], DS_FP[2][int(m/2):int(m/2+2)], marker = "o",color = "red") 
            ax1.scatter(DS_FP[0][int(m/2+2):], DS_FP[1][int(m/2+2):], DS_FP[2][int(m/2+2):])
            ax1.scatter(DS_FP[0][0:int(m/2)], DS_FP[1][0:int(m/2)], DS_FP[2][0:int(m/2)], marker= "X")
        else:
            ax1.scatter(DS_FP[0][-1], DS_FP[1][-1], DS_FP[2][-1], marker = "*",color = "red")
            ax1.scatter(DS_FP[0][int((m+1)/2+2):], DS_FP[1][int((m+1)/2+2):], DS_FP[2][int((m+1)/2+2):])
            ax1.scatter(DS_FP[0][0:int((m+1)/2)], DS_FP[1][0:int((m+1)/2)], DS_FP[2][0:int((m+1)/2)], marker= "X")
        
        

        # Gráficos 2D a la derecha
        
        HSpeed.append(discPoints[i].HSpeed), VSpeed.append(discPoints[i].VSpeed) 
        
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(time, HSpeed, color = "black")
        ax2.fill_between(time, HSpeed, color = "blue", alpha = .7)
        ax2.set_title(f'Horizontal Accel {round(discPoints[i].HAccel,2)} m/s^2')
        ax2.set_ylabel("Horizontal Speed (m/s)")
        ax2.set_ylim(0-0.5, discretization.drone.horizontalSpeed+.5)

        
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(time, VSpeed, color = "black")
        ax3.fill_between(time, VSpeed, color = "blue", alpha = .7)
        ax3.set_title(f'Vertical Accel {round(discPoints[i].VAccel,2)} m/s^2')
        ax3.set_ylabel("Vertical Speed (m/s)")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylim(-discretization.drone.verticalSpeed-.5, discretization.drone.verticalSpeed+.5)
       
        
        plt.tight_layout()
        
        filename = f"{output_dir}/frame_{i}.png"
        plt.savefig(filename)
        images.append(imageio.v3.imread(filename))
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
        axisTicks[0][0]= min(dataSet[0]+axisTicks[0][0])
        axisTicks[0][1]= max(dataSet[0]+axisTicks[0][1])
        axisTicks[1][0]= min(dataSet[1]+axisTicks[1][0])
        axisTicks[1][1]= max(dataSet[1]+axisTicks[1][1])
        axisTicks[2][0]= min(dataSet[2]+axisTicks[2][0])
        axisTicks[2][1]= max(dataSet[2]+axisTicks[2][1])


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



