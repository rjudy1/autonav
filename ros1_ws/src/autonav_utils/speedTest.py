#!/usr/bin/env python
import geopy.distance

#this program will look over saved GPS data and will calculate the average speed that the robot traveled at

with open("/home/autonav/autonav_ws/src/gps_connection/src/gpsData.csv", "r") as GPS_Data:
    lines = GPS_Data.readlines()

print()
time = []
#convert lat and long
for i in range(1,len(lines)-1):
    cell1 = lines[i].split(",")
    cell2 = lines[i+1].split(",")
    lat1 = cell1[0]
    lat2 = cell2[0]
    long1 = cell1[1]
    long2 = cell2[1]
    coords_1 = (float(lat1), float(long1))
    coords_2 = (float(lat2), float(long2))
    distance = geopy.distance.distance(coords_1, coords_2).mi
    time.append(float(distance/5.55556e-5))

print("Average speed: " + str(sum(time)/len(time)) + " mi/hr")

GPS_Data.close()
   