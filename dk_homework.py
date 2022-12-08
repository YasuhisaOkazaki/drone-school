from dronekit import connect, LocationGlobal, LocationGlobalRelative, VehicleMode
import time
import math

#------------------------
# 設定（変数など）
#------------------------
# connect
connect_msg = 'tcp:127.0.0.1:5762'
# takeoff高度 (m)
takeoff_alt = 10
# ground speed (m/s)
groundspeed = 5
# 相対座標リスト (m)：前の座標（最初は離陸地点）からの相対
'''
dict_position = {
    '1' : {'North': 80, 'East': -50},
    '2' : {'North': 0, 'East': 100},
    '3' : {'North': -80, 'East': -50},
}
'''
dict_position = {
    '1' : {'North': 0, 'East': -50},
    '2' : {'North': -50, 'East': 0},
    '3' : {'North': 0, 'East': 50},
    '4' : {'North': 50, 'East': 0},  
}
# 動作間のwait：離陸と座標移動動作の間、座標移動とRTL動作の間
wait_time = 1
# 目標距離許容値？（目標座標までの距離＜（移動開始時の座標までの距離 * targetDistance_mul））
# wikiでは0.01だったが、移動距離が小さいと無限loopに陥る可能性がある
#（移動距離：100m前後の場合0.01、以下調整必要）
targetDistance_mul = 0.015
# 以下、万が一の無限loop防止の処置
# groundspeed： stop_groundspeed(m/s)未満が連続してstop_time(s)続いた時breakする
stop_groundspeed = 0.1
stop_time = 5 # 1<の値



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
def goto(dNorth, dEast):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    #targetLocation.lat = round(targetLocation.lat, 7)
    #targetLocation.lon = round(targetLocation.lon, 7)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    #gotoFunction(targetLocation)
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



#------------------------
# main
#------------------------
# connect
vehicle = connect(connect_msg, wait_ready=True, timeout= 60)

# Takeoff
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    print("Mode: GUIDED")
    vehicle.arm(wait=True, timeout=10)
    print("ARMED")
    print("Takeoff")
    vehicle.wait_simple_takeoff(takeoff_alt, timeout=20)
except TimeoutError as takeoffError:
    print("Takeoff is timeout!!!")

# wait
time.sleep(wait_time)

# Set groundspeed
vehicle.groundspeed = groundspeed
print("Set groundspeed to {}m/s.".format(groundspeed))

# goto
for key_position, value_position in dict_position.items():
    str_n = "North"
    int_n = value_position['North']
    str_e = "East"
    int_e = value_position['East']
    if int_n < 0:
        str_n = "South"
        int_n *= -1
    if int_e < 0:
        str_e = "West"
        int_e *= -1
    print("Position {}: {} {}m, {} {}m".format(key_position, str_n, int_n, str_e, int_e))
    goto(value_position['North'], value_position['East'])

# wait
time.sleep(wait_time)

# RTL
vehicle.mode = VehicleMode('RTL')
print("Mode: RTL")

# 終了処理
while vehicle.armed:
    time.sleep(1)
print("DISARMED")
vehicle.close()
print("Connect Close")
print("Finished!")