from dronekit import connect, LocationGlobal, LocationGlobalRelative, VehicleMode
import time
import math
import sys

#------------------------
# 設定（変数など）
#------------------------
# connect
# connect_msg = 'tcp:127.0.0.1:5782'
#connect_msg = 'tcp:127.0.0.1:5760'
connect_msg = 'tcp:127.0.0.1:5762'

# 座標(開始ポイント)
start_position = {
    'lat': 335.879768,
    'lon': 140.348495
}
# 相対座標リスト (m)：前の座標（最初は離陸地点）からの相対
dict_position = {
    '1' : {'lat': 35.8797360, 'lon': 140.3484026},
    '2' : {'lat': 35.8795731, 'lon': 140.3485152},
    '3' : {'lat': 35.8794361, 'lon': 140.3485394},
    '4' : {'lat': 35.8793514, 'lon': 140.3484978},
    '5' : {'lat': 35.8790732, 'lon': 140.3482658},
    '6' : {'lat': 35.8790026, 'lon': 140.3482403},
    '7' : {'lat': 35.8788472, 'lon': 140.3483422},
    '8' : {'lat': 35.8786538, 'lon': 140.3480498},
    '9' : {'lat': 35.8779279, 'lon': 140.3488934},
}
# 座標(終端ポイント)
end_position = {
    'lat': 35.876991,
    'lon': 140.348026
}
dict_position_b = {
    '9' : {'lat': 35.8779279, 'lon': 140.3488934},
    '8' : {'lat': 35.8786538, 'lon': 140.3480498},
    '7' : {'lat': 35.8788472, 'lon': 140.3483422},
    '6' : {'lat': 35.8790026, 'lon': 140.3482403},
    '5' : {'lat': 35.8790732, 'lon': 140.3482658},
    '4' : {'lat': 35.8793514, 'lon': 140.3484978},
    '3' : {'lat': 35.8794361, 'lon': 140.3485394},
    '2' : {'lat': 35.8795731, 'lon': 140.3485152},
    '1' : {'lat': 35.8797360, 'lon': 140.3484026},
}

# 動作間のwait：離陸と座標移動動作の間、座標移動とRTL動作の間
wait_time = 1

# 以下、万が一の無限loop防止の処置
# groundspeed： stop_groundspeed(m/s)未満が連続してstop_time(s)続いた時breakする
stop_groundspeed = 0.1
stop_time = 5 # 1<の値
targetDistance_mul = 0.1


#------------------------
# 関数
# wiki等から(gotoのみ一部改変)
#------------------------
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.
    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

# 以下は、wikiの一部改変（第三引数の部分および移動停止時の処置）
#def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
def goto(dLat, dLon, beforeLocation):
    if beforeLocation is None:
        currentLocation = vehicle.location.global_relative_frame
    else:
        currentLocation = beforeLocation
    targetLocation = get_location_metres(currentLocation, dLat, dLon)
    targetLocation.lat = dLat
    targetLocation.lon = dLon
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)
    
    #print "DEBUG: targetLocation: %s"
    #print("targetDistance * targetDistance_mul: ", targetDistance*targetDistance_mul)
    #print("DEBUG: targetLocation: %s" % targetDistance)
    chkflag = 0
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        # print("vehicle.groundspeed:", vehicle.groundspeed)
        if remainingDistance<=targetDistance*targetDistance_mul: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        # 途中で停止した場合の処置（stop_time秒groundspeedがstop_groundspeed未満の場合break）
        if vehicle.groundspeed < stop_groundspeed:
            if chkflag >= stop_time:
                print("groundspeed < {}  {}s : break".format(stop_groundspeed, stop_time))                
                break;
            chkflag += 1
        time.sleep(1)
    return beforeLocation


#------------------------
# main
#------------------------
# connect
vehicle = connect(connect_msg, wait_ready=True, timeout= 60)

# コネクト
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    print("Mode: GUIDED")
    vehicle.arm(wait=True, timeout=10)
    print("ARMED")
except TimeoutError as takeoffError:
    print("timeout!!!")
    sys.exit()

# goto
mode_goto = False # False: start->end True: End->Start
while True:
    beforeLocation = None
    if mode_goto:
        l_dict_position = dict_position_b
    else:
        l_dict_position = dict_position
    for key_position, value_position in l_dict_position.items():
        beforeLocation = goto(value_position['lat'], value_position['lon'], beforeLocation)
    if mode_goto:
        l_position = start_position
    else:
        l_position = end_position
    beforeLocation = goto(l_position['lat'], l_position['lon'], beforeLocation)
    if mode_goto:
        mode_goto = False
    else:
        mode_goto = True
    time.sleep(10)

# RTL
#vehicle.mode = VehicleMode('RTL')
#print("Mode: RTL")
#vehicle.close()
#print("Connect Close")

