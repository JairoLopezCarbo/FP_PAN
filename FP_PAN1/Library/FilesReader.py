import pandas as pd
import os
import numpy as np
from Library.LatLonProjection import Point

def CsvPointListReader(filePath):
    """CSV FILE READER:
        FlightPlan: lat, lon & alt [m]
        Telemetry: lat, lon, alt [m] & time [s]
    
    Args:
        filePath (string): filePath.csv
    Returns:
        List<Point>: Point List
    """
    data = ["lat", "lon", "alt", "secs"]
    df = pd.read_csv(filePath)
    for i in range(len(data)):
        for column in df.columns:
            if data[i] in column:
                data[i] = column

    lat =  df[data[0]].to_numpy()
    lon = df[data[1]].to_numpy()
    alt = df[data[2]].to_numpy()
    try:
        time = df[data[3]].to_numpy()
    except:
        time = np.zeros(len(alt))
        
    pointList = []
    for i in range(len(lat)):
        point = Point(lat[i], lon[i], alt[i], time =time[i])
        pointList.append(point)
    return pointList

def CsvAvailableDataReader(folderPath):
    """Asign FlightPlan & Telemetry Data to Drones
    
    Args:
        folderPath (string): data folderPath
    Returns:
        Dictionary <drone(string) : dataFile(string)>: Dictionary with drone key and all data available
    """
    droneData = {}
    df = pd.read_csv(folderPath)
    FP_Files = os.listdir('Data/FlightPlan/')
    Telem_Files = os.listdir('Data/Telemetry/')
    for i in range(len(df["drone"])):
        if not("FlightPlan_"+df["id"][i]+".csv" in FP_Files) or not("Telemetry_"+df["id"][i]+".csv" in Telem_Files):
            continue
        if df["drone"][i] in droneData.keys():
            droneData[df["drone"][i]].append(df["id"][i])
        else:
            droneData.update({df["drone"][i]: [df["id"][i]]})
    return droneData