from __future__ import print_function
import time
import random
import numpy
import psutil
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative


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
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    x=0
    while True:
        print("Source Points: ", vehicle.location.global_relative_frame.alt)
        x=x+1
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt:
            print("Reached target altitude")
            break
        if x==20:
            break
        time.sleep(1)
arm_and_takeoff(10)

print("Going towards Destination points ...")

# Fixed_Inputs
depo_x = 0
depo_y = 0
avg_e_com = 40  # (wh/km):

drone1_x = 9
drone1_y = 10
drone1_e = 100  # (wh) Energy
demand1_x = 11  # units in meters
demand1_y = 13 # units in meters


# calculations
d1 = math.sqrt((demand1_x - drone1_x) ** 2 + (demand1_y - drone1_y) ** 2)
#print(d1)
d_demand_depo = math.sqrt((demand1_x - depo_x) ** 2 + (demand1_y - depo_y) ** 2)
#print(d_demand_depo)
d_depo = math.sqrt((drone1_x - depo_x) ** 2 + (drone1_y - depo_y) ** 2)
#print(d_depo)
e1 = d1 * avg_e_com / 1000  # (distance km)
#print(e1)
e_depo = d_demand_depo * avg_e_com / 1000  # (distance km)
#print(d_demand_depo)
e_total = e1 + d_demand_depo
#print(e_total)

while True:
  out_1 = drone1_x+drone1_y
  out_2 = demand1_x+demand1_y
  values = [out_1]
  values_2 = [out_2]
  out =out_1+out_2
  currentTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
  demand_points_1 = random.randint(9,10)
  demand_points_2 = random.randint(11,12)
  time.sleep(5)
  if  ((demand_points_1 == 9) and (demand_points_2 == 12)):
      break
  print(currentTime,demand_points_1,demand_points_2)
items = []


def getEquidistantPoints(demand1_x , demand1_y , parts):
    return zip(numpy.linspace(demand1_x[0], demand1_y[0], parts+1),
               numpy.linspace(demand1_x[1], demand1_y[1],parts+1))
print(list(getEquidistantPoints((9,9), (13,13),10)))

def get_distance_between_coordinates(sourcepoints,demandspoints):
    d1 = math.sqrt((demandspoints[0] - sourcepoints[0]) ** 2 + (demandspoints[1] - sourcepoints[1]) ** 2)
    return d1

def call_battery_fun(getEquidistantPoints,avg_e_com):
    e1 = getEquidistantPoints * avg_e_com / 100
    return e1

list_of_coordinates=list(getEquidistantPoints((9,9), (13,13),10))
l = len(list_of_coordinates)
print(l)
for i in range(len(list_of_coordinates)):
    if i + 1 == l:
        break
    else:
        distnce = get_distance_between_coordinates(list_of_coordinates[i], list_of_coordinates[i+1])
        battery_percentage = call_battery_fun(distnce, 40 )
        print("Distance : " + str(distnce) + "  Energy(%):  " + str(battery_percentage))

print("Drone Reached the required Destination point")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
