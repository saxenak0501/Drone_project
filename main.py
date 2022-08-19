from numpy import sqrt
import random
import pandas as pd
import numpy as np
import string
import math
import time
from time import sleep
import threading
from datetime import datetime, timedelta as delta
import numpy
from argparse import ArgumentParser
from time import sleep
from threading import Timer
from functools import partial
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse
import socket


# Fixed_Inputs
depo_x = 0
depo_y = 0
avg_e_com = 40  # (wh/km):

# Variable_Inputs
drone1_x = 20
drone1_y = 30
drone1_e = 100  # (wh) Energy
demand1_x = 500  # units in meters
demand1_y = 1000  # units in meters

# calculations
d1 = math.sqrt((demand1_x - drone1_x) ** 2 + (demand1_y - drone1_y) ** 2)
print(d1)
d_demand_depo = math.sqrt((demand1_x - depo_x) ** 2 + (demand1_y - depo_y) ** 2)
print(d_demand_depo)
d_depo = math.sqrt((drone1_x - depo_x) ** 2 + (drone1_y - depo_y) ** 2)
print(d_depo)
e1 = d1 * avg_e_com / 1000  # (distance km)
print(e1)
e_depo = d_demand_depo * avg_e_com / 1000  # (distance km)
print(d_demand_depo)
e_total = e1 + d_demand_depo
print(e_total)

drone1_x = demand1_x
drone1_y = demand1_y
drone1_e = drone1_e - e1
demand1_x = 0
demand1_y = 0
if e_total < drone1_e:
   print(drone1_x, drone1_y)

else:
    print(drone1_x,drone1_y)

drone1_x_1 = demand1_x
drone1_y_1 = demand1_y
drone1_e = drone1_e - e1
demand1_x = 0
demand1_y = 0
if e_total == drone1_e:
    print(drone1_x_1,drone1_y_1)
    print(e_total)

drone1_x = depo_x
drone1_y = depo_y
drone1_e = drone1_e - d_depo * avg_e_com / 1000  # (distance km)
if drone1_e < 100: #(10% of Batetry of drone)
   print(drone1_e)
   print(drone1_x,drone1_y)


#while True:
 # out_1 = drone1_x+drone1_y
  #out_2 = demand1_x+demand1_y
  #values = [out_1]
  #values_2 = [out_2]
  #currentTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
  #demand_points_1 = random.uniform(50,200)
  #demand_points_2 = random.uniform(200,400)
  #time.sleep(2)
#print(test)

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
vehicle = connect('udp:127.0.0.1:14551')

def get_location_points(location, demand_1, demand_2):
    fixed_cal=6371.0
    Lat = demand_1/fixed_cal
    Lon = demand_2/(fixed_cal*math.cos(math.pi*location.lat/180))
    lat_1 = location.lat + (Lat * 180/math.pi)
    lon_1 = location.lon + (Lon * 180/math.pi)
    return LocationGlobal(lat_1, lon_1,location.alt)

def get_distance_metres(demand_x, demand_y):
    demand_1 = demand_y.lat - demand_x.lat
    demand_2 = demand_y.lon - demand_x.lon
    return math.sqrt((demand_1*demand_1) + (demand_2*demand_2))

def distance_to_current_waypoint():
    nextpoint = vehicle.commands.next
    if nextpoint==0:
        return None
    mission_points=vehicle.commands[nextpoint-1]
    latt = mission_points.x
    long = mission_points.y
    altt = mission_points.z
    targetLocation = LocationGlobalRelative(latt,long,altt)
    distancepoint = get_distance_metres(vehicle.location.global_frame, targetLocation)
    return distancepoint

def download_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def mission(aLocation, Size):
    cmds = vehicle.commands
    print(" Clear any existing commands")
    cmds.clear()
    print(" Define/add new commands.")
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    point_1 = get_location_points(aLocation, Size, -Size)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point_1.lat, point_1.lon, 11))
    print(" Upload new commands to vehicle")
    cmds.upload()

def arm_and_takeoff(Altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(Altitude)
    x=0
    while True:
        print("Source Points: ", vehicle.location.global_relative_frame.alt)
        x=x+1
        if vehicle.location.global_relative_frame.alt:
            break
        if x==20:
            break
        time.sleep(1)

print('Create a new mission (for current location)')
mission(vehicle.location.global_frame,50)
arm_and_takeoff(1)

print("Starting mission")
# Reset mission set to first waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

x=0
while True:
    nextpoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextpoint, distance_to_current_waypoint()))
    x=x+1
    if x==3:
       break
    time.sleep(1)

print('Return to launch')
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
