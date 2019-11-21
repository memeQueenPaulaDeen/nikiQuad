
import time, argparse, datetime
from datetime import datetime
import struct, math
import numpy as np
#import transformations
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil, quaternion
import socket
import threading
import os
import ctypes
#import droneapi.lib
#from droneapi.lib import VehicleMode
#from droneapi.lib import Location

import datetime
import time

# Parameter That shows whether the drone is outdoor or indoor

missionIsIndoor = True 
LEDisEquipped = False
untrackedSeqNo = 0

#LOCAL SERVER SENDS
x = [0]
y = [0]
z = [0]
yawAng = [0]
pitchAng = [0]
rollAng = [0]
isTracking = [0]
lastZ = [0]
lastY = [0]
lastX = [0]
lastTime = [0]

#AUTOPILOT SENDS
ax = [0]
ay = [0]
az = [0]
ayawAng = [0]
apitchAng = [0]
arollAng = [0]
rcInChannel  = [0,0,0,0]
servoOut  = [0,0,0,0,0,0,0,0]
flightMode = [0] # 0: Unset / 1: Stabilize / 2: Alt Hold / 3: Loiter / 4: Auto / 5: Guided / 6: Land / 10: Unknown
manualControlMode = [-1]

#RANGEFINDER
rangefinderDistance = [0]
# Autopilot Altitude Value
autopilotAltitude = [0]
#SEND TO LED SERVER
enableLED = [1] 
batState = [0]
rcState = [0] # 1 is connected
autopilot = [0]
gpsState = [0] #See referance ^^
#indoorOutdoor = [0] # 1 Indoor
autoland = [0] # 1 is true
lockVar = threading.Lock()
lockVarWrite = threading.Lock()

#DECIDE CONNECTION TO LOCAL SERVER
localConnect = False


#GPS2 HDOP GLOBAL VAR
ghdop2 = [0]

homeLat = 36.1500053181808
homeLong = -95.9441225230694
homeAlt = 0

class Drone(): #Connects to dronekit vehicle

    def __init__(self, connection_string = None, vehicle = None):
        if not connection_string is None:
            print("Connecting with vehicle...")
            self.vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60,baud=921600)
            self.vehicle.wait_ready(True,timeout=180)
            
        else:
            print("No connection string")
            return
            
def arm_and_takeoff(self, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not self.vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    self.vehicle.mode = VehicleMode("GUIDED")
    self.vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not self.vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", self.vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
            break
        time.sleep(1)

def addWaypoint(lat, lon, alt):
     return( Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))



#-----------------------Autopilot Communication------------------------


#Connects to autopilot and executes AutopilotRead() and AutopilotWrite()
def AutopilotCommunication():

    global IMThread
    global OMThread
    
    autopilot = [0] #Initial value for connect
    
    parser = argparse.ArgumentParser(description='Drone connect options')
    parser.add_argument('--connect', default = 'tcp:127.0.0.1:5760', help="Vehicle connection target string")

    args = parser.parse_args()
    Us = 10**6
    #Connect to vehicle
    connection_string = args.connect # for SITL or Mavlink Router Connection
    #connection_string = '/dev/ttyS0' # for Serial Connection
    print("Connecting to vehicle on: %s" % (connection_string,))
    AutopilotCommunication.drone = Drone(connection_string)

    AutopilotCommunication.timeSyncReceived = False
    AutopilotCommunication.gpsGlobalOriginSet = False
    AutopilotCommunication.timeDiff = 0
    AutopilotCommunication.autopilotTime = 0
    AutopilotCommunication.tic= 0
    
   
    @AutopilotCommunication.drone.vehicle.on_message('TIMESYNC') # Performs Time Sync
    def TimeSyncFunc(self, name, msg):

        #global tic 
        AutopilotCommunication.tic = int(round(time.time() * Us)) - (msg.ts1 %1000)
        #global autopilotTime
        AutopilotCommunication.autopilotTime   = msg.ts1 /1000
        #print ('autopilotTime = %.1f'% float(AutopilotCommunication.autopilotTime))
        #print msg
        #print " TimeSync Performed."
        #global timeSyncReceived 
        AutopilotCommunication.timeSyncReceived = True
        #print "AUTOPILOT ON"
        autopilot = [1]
        
    while AutopilotCommunication.timeSyncReceived == False: # Wait for Time Sync to happen
        myTime= int(round(time.time() * Us))
        AutopilotCommunication.drone.vehicle.message_factory.timesync_send(myTime, 0)
        print ("wait for TimeSync...")
        time.sleep(1)

       
    threading.Thread(target = AutopilotReadMgr).start()

    if missionIsIndoor == True:
        threading.Thread(target = AutopilotIndoorPositionWriteMgr).start()
        IMThread.start()
    else:
        OMThread.start()
        
    #threading.Thread(target = FlightModeMgr).start()
        
def AutopilotReadMgr():
    
    while True:
        AutopilotRead()
        time.sleep(0.05) # 20Hz autopilot data Update


def AutopilotIndoorPositionWriteMgr():
    
    while True:
        AutopilotIndoorPositionWrite()
        time.sleep(0.02) # 16Hz location update
        #time.sleep(0.1) # 10Hz location update

def FlightModeMgr():
    
    global IMThread
    global OMThread
    global flightMode
    global untrackedSeqNo
    mgrLoop = True
    
    while mgrLoop == True:  
           
        if missionIsIndoor == True:
            
            if manualControlMode == [1]:
                
                enableCompass()
                if (flightMode == [1]) or (flightMode == [2]) or (flightMode == [6]):
                #don't change Filght Mode 
                    pass
                else:
                    #if connected to auto pilot
                    if (flightMode <> [0]): 
                        #IMThread.join()
                        AutopilotCommunication.drone.vehicle.mode = VehicleMode("LAND")
                        print "Force Landing in Manual Mode!"
                        mgrLoop = False
            elif manualControlMode == [0]:
                
                if (untrackedSeqNo < 10):
                                       
                    pass
                else:
                    #If connected to autopilot and localization Server
                    if (flightMode != [0]) and (localConnect == True):
                        
                        enableCompass()
                        # IMThread.join()
                        AutopilotCommunication.drone.vehicle.mode = VehicleMode("LAND")
                        print "Force Landing in Auto Mode!"
                        mgrLoop = False
        else: # Outdoor Mission Strategy
            pass # Change it for actual outdoor Mission
        time.sleep (0.01)
                    

        
def AutopilotRead():
    
    global enableLED
    global batState
    global rcState
    global autopilot
    global gpsState
    global autoland
    global ghdop2
    global rcInChannel 
    global servoOut  
    global flightMode
    global manualControlMode
    global ayawAng
    global apitchAng
    global arollAng
    global autopilotAltitude
    
#######################  BATTERY   #############################

    numCells = 6 ### Number of battery cells in the drone
    voltage = AutopilotCommunication.drone.vehicle.battery.voltage
    maximum = numCells*4.2
    minimum = numCells*3.2
    totalBat = [int(round(100 * ((voltage-minimum) / (maximum-minimum)))) ]
    Assign(batState, totalBat)
    #print "Battery State: " + str(batState)




#######################  RC  ###################################
   
   
    AutopilotRead.rcThreshold = 975 ### Threshold for the throttle (channel 3) on the RC for it to be on. Minimum val.
    AutopilotRead.enableLEDThreshold = 1000 ##Threshold for casting the "enableLED parameter to the LED server

    @AutopilotCommunication.drone.vehicle.on_message('SERVO_OUTPUT_RAW')
    def listener(self, name, message):
        
        global servoOut
        #print ("Servo Data Received" +  str (message))   
        servoOut[0] = int (message.servo1_raw)
        servoOut[1] = int (message.servo2_raw)
        servoOut[2] = int (message.servo3_raw)
        servoOut[3] = int (message.servo4_raw)
        servoOut[4] = int (message.servo5_raw)
        servoOut[5] = int (message.servo6_raw)
        servoOut[6] = int (message.servo7_raw)
        servoOut[7] = int (message.servo8_raw)

    @AutopilotCommunication.drone.vehicle.on_message('RC_CHANNELS')
    def listener(self, name, message):
        global rcState
        global rcInChannel
        global manualControlMode
        
        # RC Signal is delivered to autopilot
        if int(message.chan3_raw) < AutopilotRead.rcThreshold:
            Assign(rcState, [0])
        else:
            Assign(rcState, [1])
        
        #Enable LED conditional. Based on RC Channel
        if int(message.chan14_raw) < AutopilotRead.enableLEDThreshold:
            Assign(enableLED, [1])
        else:
            Assign(enableLED, [0])
        
        # Autonomous / Manual Switch
        if int(message.chan6_raw) > 1500:
            Assign (manualControlMode,[1])
        else:
            Assign (manualControlMode,[0])
            
        rcInChannel[0] = int (message.chan1_raw)
        rcInChannel[1] = int (message.chan2_raw)
        rcInChannel[2] = int (message.chan3_raw)
        rcInChannel[3] = int (message.chan4_raw)


######################  AUTOPILOT COMM  ###########################


    if AutopilotCommunication.drone.vehicle.last_heartbeat > 2.5:
        Assign(autopilot, [0])
    
    else:
        Assign(autopilot, [1])
    
    #print str(AutopilotCommunication.drone.vehicle.mode)


########################  FLIGHT MODE  ###################################
# 0: Unset / 1: Stabilize / 2: Alt Hold / 3: Loiter / 4: Auto / 5: Guided / 6: Land / 10: Unknown
    
    
    if AutopilotCommunication.drone.vehicle.mode == VehicleMode("STABILIZE"):
        Assign(flightMode,[1])
        Assign(autoland, [0])
    elif AutopilotCommunication.drone.vehicle.mode == VehicleMode("ALT_HOLD"):
        Assign(flightMode,[2])
        Assign(autoland, [0])
    elif AutopilotCommunication.drone.vehicle.mode == VehicleMode("LOITER"):
        Assign(flightMode,[3])
        Assign(autoland, [0])
    elif AutopilotCommunication.drone.vehicle.mode == VehicleMode("AUTO"):
        Assign(flightMode,[4])
        Assign(autoland, [0])
    elif AutopilotCommunication.drone.vehicle.mode == VehicleMode("GUIDED"):
        Assign(flightMode,[5])
        Assign(autoland, [0])
    elif AutopilotCommunication.drone.vehicle.mode == VehicleMode("LAND"):
        Assign(flightMode,[6])
        Assign(autoland, [1])
    else:
        Assign(flightMode,[10])
        Assign(autoland, [0])
        print "Unknown Flight Mode:" + str(flightMode) 

######################## GPS #######################

    gps_type = AutopilotCommunication.drone.vehicle.parameters['EK2_GPS_TYPE']
    hdop = AutopilotCommunication.drone.vehicle.gps_0.eph
    @AutopilotCommunication.drone.vehicle.on_message('GPS2_RAW')
    def SetHDOP2(self, name, message):
        global ghdop2
        hdop2 = [message.eph]
        Assign(ghdop2, hdop2)
        
    FindGPSState(gps_type, hdop, ghdop2[0])
    
#############  Autopilot yaw, pitch, roll  ######################

    tempYawVal = np.degrees(AutopilotCommunication.drone.vehicle.attitude.yaw)
    if tempYawVal < 0:
        tempYawVal = 360 + tempYawVal
    Assign(ayawAng,  [tempYawVal])
    Assign(apitchAng, np.degrees([AutopilotCommunication.drone.vehicle.attitude.pitch]))
    Assign(arollAng, np.degrees([AutopilotCommunication.drone.vehicle.attitude.roll]))
    
    #print ("Yaw: " + str (ayawAng) + " - Pitch: " + str(apitchAng) + " - Roll: " + str(arollAng) + "\n" )

#############  Autopilot RangeFider Distance Value  ######################

    
    rngFndDist =  AutopilotCommunication.drone.vehicle.rangefinder.distance 
    if str(rngFndDist) == "None":
        rngFndDist = 0  
    Assign(rangefinderDistance, [rngFndDist])
    
#############  Autopilot Altitude Value  ######################

    autoAltVal = AutopilotCommunication.drone.vehicle.location.global_relative_frame.alt 
    Assign (autopilotAltitude,[autoAltVal])


def FindGPSState(gps_type, hdop, hdop2):
    global gpsState
    global localConnect
    global isTracking
    
    
    if localConnect: #Indoor Mode
        
        if isTracking == [1]: #Local Server is Tracking /  {solid purple}
            Assign(gpsState, [1])
        else: #Local Server not Tracking / {blinking pink}
            Assign(gpsState, [2])
            
    else: #Outdoor Mode
        if gps_type == 3: #GPS not active / {blinking yellow}
            Assign(gpsState, [3])
        else: #GPS active
            if hdop<50 or hdop2<50 : #accurate location / {solid green}
                Assign(gpsState, [4])
            elif hdop<100 or hdop2<100 : #somewhat accurate / {blinking green}
                Assign(gpsState, [5])
            else: #not accurate / {solid yellow}
                Assign(gpsState, [6])
    ####################  gpsState REFERANCE  ########################
    #GPS STATES (gpsState):
    ###(0) Initial - VALUE 0 : LEDS OFF
    ###(1) Local Server Connected, isTracking, - VALUE 1 : SOLID PURPLE
    ###(2) Local Server Connected, no isTracking - VALUE 2 : BLINKING PINK
    ###(3) Local Server Disconnected, Outdoor GPS Disabled - VALUE 3 : BLINKING YELLOW
    ###(4) Local Server Disconnected, Outdoor GPS Enabled, HDOPs < .5 - VALUE 4 : SOLID GREEN
    ###(5) Local Server Disconnected, Outdoor GPS Enabled, HDOPs < 1 - VALUE 5 : BLINKING GREEN
    ###(6) Local Server Disconnected, Outdoor GPS Enabled, HDOPs > 1 - VALUE 6 : SOLID YELLOW
    ##################################################################


    #Callback to print the location
        #def location_callback(location):
        #   print "Location: ", AutopilotCommunication.drone.vehicle.location

    #Conditionals for GPS State (gpsState value 1-6)

def disableCompass():
    
    
    while (AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE']  != 0) and ( AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE2'] != 0) and (AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE3'] != 0):
        try:
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE']  = 0
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE2'] = 0
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE3'] = 0
            time.sleep (0.5)
        except Exception,e:
            print "Disabling Compass failed:"
            print str(e)
            print "Will try again in 0.5 Sec..."
            time.sleep(0.5)
        
    print "Compass is Disabled"
    
def enableCompass():
    
    while (AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE']  != 1) and ( AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE2'] != 1) and (AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE3'] != 1):
        try:
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE']  = 1
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE2'] = 1
            AutopilotCommunication.drone.vehicle.parameters['COMPASS_USE3'] = 1
            time.sleep (0.5)
        except Exception,e:
            print "Enabling Compass failed:"
            print str(e)
            print "Will try again in 0.5 Sec..."
            time.sleep(0.5)
            
    print "Compass is Enabled"
    
def getLat(x):
    #pass x in as meters
    return (((180.0 / (np.pi * 6371008.8) * x) + homeLat))
    
def getLong(y):
    #pass y in as meters
    return ((homeLong + (180.0 / (np.pi * 6371008.8) * (y))))
        
#Writes values to send to autopilot
def AutopilotIndoorPositionWrite():
    
    global x
    global y
    global z
    global yawAng
    global pitchAng
    global rollAng
    global isTracking
    global flightMode
    global manualControlMode
    global untrackedSeqNo

    global lastZ
    global lastY
    global lastX
    global lastTime

    
    #local instances of global variables
    lx = [0]
    ly = [0]
    lz = [0]
    lyawAng = [0]
    lpitchAng = [0]
    lrollAng = [0]


    Us = 10**6
    
    #local var = global var
    Assign(lx, x)
    Assign(ly, y)
    Assign(lz, z)
    Assign(lyawAng, yawAng)
    Assign(lpitchAng, pitchAng)
    Assign(lrollAng, rollAng)


    if manualControlMode == [1]: # Remote Control Channel 6 is in Manual Mode
        
        pass # don't send position information to autopilot
            
    else: # Remote Control Channel 6 is in Autonumous Mode
        
        if (localConnect == True):
            
            
            #Changing location system from euler to quaternion
            q = quaternion.QuaternionBase([np.radians(float(lrollAng[0])), np.radians(float(lpitchAng[0])), np.radians(float(lyawAng[0]))])
            toc = int(round(time.time() * Us))+999 #Timestamp declaration
            timeDiff = toc - AutopilotCommunication.tic;
            tStamp = AutopilotCommunication.autopilotTime + timeDiff
            # Setting Home (origin) Global Position
            #init_gps_pos = [36.1500053181808, -95.9441225230694,0] #Initial position: Maybee Pool Lab
            if AutopilotCommunication.gpsGlobalOriginSet == False:
                
                homeAlt = AutopilotCommunication.drone.vehicle.location.global_relative_frame.alt
                message_origin = AutopilotCommunication.drone.vehicle.message_factory.set_gps_global_origin_encode(tStamp, int(homeLat*1e7),
                                                                             int(homeLong*1e7),
                                                                             int(homeAlt*1000))
                AutopilotCommunication.drone.vehicle.send_mavlink(message_origin)
                time.sleep(1)
                AutopilotCommunication.gpsGlobalOriginSet = True
                print "GPS Global Origin is Set" 
                time.sleep(1)
                #disableCompass()
                
                    
            if isTracking == [1]:
                
            # MOCAP MSG Approach
            
                #print (" Sending Pos Data To AutoPilot: " + " X: " + str(lx[0])+ " - Y: " + str(ly[0])+ " - Z: " + str(lz[0])+ " - Yaw: " + str(float(lyawAng[0]))  )
                #AutopilotCommunication.drone.vehicle.message_factory.att_pos_mocap_send(tStamp, q, lx[0], ly[0], lz[0])
                
            # GPS MSG Approach
            
                alt_amsl = -lz[0]
                #lat = (((180.0 / (np.pi * 6371008.8) * lx[0]) + init_gps_pos[0])*10000000)
                lat = getLat(lx[0]) * 1e7
                #lon = ((init_gps_pos[1] + (180.0 / (np.pi * 6371008.8) * (ly[0])))*10000000)
                lon = getLong(ly[0]) * 1e7

                fix_type        = 3
                sat_visible     = 20
                epoch           = 316029582
                cur             = datetime.datetime.utcnow()
                ts              = cur - datetime.datetime(1970,1,1)
                epoch_sec       = ts.total_seconds() - epoch
                time_week       = epoch_sec / 604800.0
                time_week_ms    = (epoch_sec % 604800) * 1000.0 + ts.microseconds*0.001
                ignore_flags = 224
                hdop = .1
                vdop = 0.1 # ignored
                vert_acc = 0#ignored
                spd_acc = .1
                hor_acc = .1

                vn = 0
                ve = 0
                vd = 0
                
                #print "llastTime: " + str(lastTime[0])
            
                if lastTime[0] == 0:
                    vn = 0
                    ve = 0
                    vd = 0
                    lastTime[0] = cur
                    #print "cur: " + str(cur)
                else:
                    #print type(cur)
                    #print type(lastTime[0])
                    vts = (cur-lastTime[0]).microseconds
                    vd = -((alt_amsl - lastZ[0])/vts)*(1000000)
                    vn = ((lx[0] - lastX[0])/vts)*(1000000)
                    ve = ((ly[0] - lastY[0])/vts)*(1000000)
                    #print "vn: " + str(vn)
                    #print "vd: " + str(vd)
                    #print "ve: " + str(ve)
                    #print 
                    
                    lastTime[0] = cur

                lastX[0] = lx[0]
                lastY[0] = ly[0]
                lastZ[0] = alt_amsl

                # UNIT16_MAX = 65535
                #
                # msg = AutopilotCommunication.drone.vehicle.message_factory.GPS2_RAW(
                #                         tStamp,
                #                         fix_type,
                #                         int(lat * 1e7),
                #                         int(lon * 1e7),
                #                         alt_amsl,
                #                         UNIT16_MAX,
                #                         UNIT16_MAX,
                #                         UNIT16_MAX,
                #                         UNIT16_MAX,
                #                         sat_visible,
                #                         sat_visible,
                #                         62)



                msg = AutopilotCommunication.drone.vehicle.message_factory.gps_input_encode(
                                     time.time()*1000000,       # time_boot_ms (not used)
                                     0,                 # gps_id
                                     ignore_flags,      # gps_flag
                                     int(time_week_ms),
                                     int(time_week),
                                     fix_type,
                                     int(lat), int(lon), alt_amsl,
                                     hdop, vdop,
                                     vn, ve, vd,
                                     spd_acc, hor_acc, vert_acc,
                                     sat_visible)
                                     
                AutopilotCommunication.drone.vehicle.send_mavlink(msg)
                              
                
                
                untrackedSeqNo = 0
                
            else:
                untrackedSeqNo+=1
        
        else:
            # don't send location data to autopilot
            pass
            

            

def outdoorMission():
    
    print "Outdoor Mission Started" 
    
def waitForPos(curX, curY, curZ, desiredX, desiredY, desiredZ, errTol):
    curPos = np.array((curX,curY,-curZ))
    desiredPos = np.array((desiredX, desiredY, desiredZ))
    delta = np.linalg.norm(desiredPos - curPos)
    #delta2 = np.sqrt((desiredX - curX)**2 + (desiredY - curY)**2 + (desiredZ - curZ)**2)
    
    print "Distance: " + str(delta)
    #print "Distance2: " + str(delta2)
    #print
    
    
    return errTol < delta

def takeOff(aTargetAltitude,err):
    print "Taking off!"
    AutopilotCommunication.drone.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", AutopilotCommunication.drone.vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if abs(AutopilotCommunication.drone.vehicle.location.global_relative_frame.alt - aTargetAltitude)<err: 
            print "Reached target altitude"
            break
        time.sleep(.1)





def indoorMission():
       
    print "Indoor Mission Started.."
    try: # write all mission code in this 'try' segment 
        #AutopilotCommunication.drone.vehicle.mode = VehicleMode("GUIDED")
        while AutopilotCommunication.drone.vehicle.mode != VehicleMode("GUIDED"):
            time.sleep(0.5)
            
    
        while not AutopilotCommunication.drone.vehicle.armed:
            print "wait for arm"
            time.sleep(1)
            
        Alt = 1.7
        AirSpeed = .5
        groundSpeed = .5 
        AutopilotCommunication.drone.vehicle.airspeed = AirSpeed
        AutopilotCommunication.drone.vehicle.groundspeed = groundSpeed
        err = .5
        
        Ax = -1.534
        Ay = 1.411
        locA = LocationGlobalRelative(getLat(Ax),getLong(Ay),Alt)
        
     
                
        Bx = -1.480
        By = -1.560
        locB = LocationGlobalRelative(getLat(Bx),getLong(By),Alt)
        
        Cx = 1.475
        Cy = -1.400
        locC = LocationGlobalRelative(getLat(Cx),getLong(Cy),Alt)
        
        Dx = 1.412
        Dy = 1.420
        locD = LocationGlobalRelative(getLat(Dx),getLong(Dy),Alt)
        
        Hx = 0
        Hy = 0
        locH = LocationGlobalRelative(getLat(Hx),getLong(Hy),Alt)
        
        
        
        # Arm
        
        # Take off
        takeOff(Alt,err)
        
        # goto A
        AutopilotCommunication.drone.vehicle.simple_goto(locA,AirSpeed)
        while waitForPos(x[0],y[0],z[0],Ax,Ay,Alt,err):
            time.sleep(.1)
            print "towards A"
        print "arrived at point A"
        # goto B
        AutopilotCommunication.drone.vehicle.simple_goto(locB,AirSpeed)
        while waitForPos(x[0],y[0],z[0],Bx,By,Alt,err):
            time.sleep(.1)
            print "towards B"
        print "arrived at point B"

        # goto C
        AutopilotCommunication.drone.vehicle.simple_goto(locC,AirSpeed)
        while waitForPos(x[0],y[0],z[0],Cx,Cy,Alt,err):
            time.sleep(.1)
            print "towards C"
        print "arrived at point C"

        # goto D
        AutopilotCommunication.drone.vehicle.simple_goto(locD,AirSpeed)
        while waitForPos(x[0],y[0],z[0],Dx,Dy,Alt,err):
            time.sleep(.1)
            print "towards D"
        print "arrived at point D"

        
        # goto H
        
        AutopilotCommunication.drone.vehicle.simple_goto(locH,AirSpeed)
        while waitForPos(x[0],y[0],z[0],Hx,Hy,Alt,err):
            time.sleep(.1)
            print "towards H"
        print "arrived at point H"
        
        # land
        print "Landing"
        AutopilotCommunication.drone.vehicle.mode = VehicleMode("LAND")
        
        
            
    finally:
        print "Indoor Mission Ended"
  
        
#Connects to LED server and sends autopilot data
def LEDServerCommunication():
    global enableLED
    global batState
    global rcState
    global autopilot
    global gpsState
    global localConnect
    global autoland
    
             # Create a socket object
    host = "10.0.0.100" #socket.gethostname() # Get local machine name
    port = 16884               # Reserve a port for your service.
    while True:
        try:
            s = socket.socket()
            c = s.connect((host, port))
            print "connected to LED Server " + host
            lenableLED = [1]
            lbatState = [80]
            lrcState = [1] # 1 is connected
            lautopilot = [0]
            lgpsState = [0] # see referance above for values
            lindoorOutdoor = [1] # 1 Indoor
            lautoland = [0] # 1 is true
            while True:
                
                Assign(lenableLED, enableLED)
                Assign(lbatState, batState)
                Assign(lrcState, rcState)
                Assign(lautopilot, autopilot)
                Assign(lgpsState, gpsState)
                Assign(lindoorOutdoor, localConnect)
                Assign(lautoland, autoland)
                    
                data = str(lenableLED[0]) +','+ str(lbatState[0]) +','+ str(lrcState[0]) +','+ str(lautopilot[0]) +','+ str(lgpsState[0]) +','+ str(lindoorOutdoor[0]) +','+ str(lautoland[0])   #raw_input("data: ")
                s.send(data)
                print data
                time.sleep(.3)
                
            
        except Exception,e:
            print "LED Conection Error: " + str(e)
            s.close()
            time.sleep(1)
            
            
#Connects to Local server, gets coordinates in Indoor environment
def LocalServerCommunication():
    global x
    global y
    global z
    global yawAng
    global pitchAng
    global rollAng
    global isTracking
    global localConnect
    
    host = "10.0.0.202" #socket.gethostname() # Get local machine name
    port = 16100   
    while True: 
        try: 
            s = socket.socket()       
            c = s.connect((host, port))
            print "Connected to Local Server " + host
            oldTime = time.time();
            while True:
                data = s.recv(1024)
                array = data.split(',')
                #print len(data)
                if len(data) > 0:
                    
                    try:
                        lx = [float(array[0])/1000.0] #Recieved value in millimeter
                        ly = [float(array[1])/1000.0] #^^
                        lz = [float(array[2])/1000.0] #^^
                        lyawAng = [float(array[3])/10.0] #Recieved value in (degree*10)
                        lpitchAng = [float(array[4])/10.0] #^^
                        lrollAng = [float(array[5])/10.0] #^^
                        
                        if str(array[6][0:1]) == "-":
                            lisTracking = [int(array[6][0:2])]
                        else:
                            lisTracking = [int(array[6][0:1])] #1 = tracking ; 0 = not tracking ; -1 = not assigned
                            
                        localConnect = True #Connected to local server
                        
                        #print ("Fresh ALT:" + str(lz)) 
                        Assign(x, lx)
                        Assign(y, ly)
                        Assign(z, lz)
                        Assign(yawAng, lyawAng)
                        Assign(pitchAng, lpitchAng)
                        Assign(rollAng, lrollAng)
                        Assign(isTracking, lisTracking)
                        
                    except Exception,e:
                        print "Error in data Conversion:"
                        print str(e)
                        
                    
                    #### TEST PURPOSES #######
                    #print "x is: " + str(x[0])
                    #print "y is: " + str(y[0])
                    #print "z is: " + str(z[0])
                    #print "yaw is: " + str(yawAng[0])
                    #print "pitch is: " + str(pitchAng[0])
                    #print "roll is: " + str(rollAng[0])
                    ##########################
                    
                    #Measure data update frequency
                    currentTime = time.time();
                    localizationServerFreq = round (1/(currentTime - oldTime));
                    oldTime = currentTime;
                    #print ("**Freq: %f" %localizationServerFreq)
                    if not data: break
                    time.sleep(.01)
                else:
                    print "No Data Received from Localization Server"
                    time.sleep(0.01)
        except Exception as e:
            print "Error: " + str(e) 
            print "Local Server Fail. Retry 1 sec ... " 
            #fileVar = open("/home/pi/DroneProject/DroneLog.txt", "r+")
            #fileVar.write(str(e))
            #fileVar.close()
            localConnect = False #Local server disconnected
            isTracking = [0]
            time.sleep(1)
            s.close()

#Threadlock-safe assignment function
def Assign(one = [], two = []):
#Syntax: Assign two To one
    global lockVar
    lockVar.acquire()
    try:
        one[0] = two[0]
    finally:
        lockVar.release()


def Logging():
    
    logDir = "/home/pi/DroneProject/LogFiles/"
    logNum = 1
    while os.path.isfile( logDir + str(datetime.date.today()) + "_LOG_" + str(logNum) + ".csv"):
        logNum += 1
        
    logFile = open(logDir+ str(datetime.date.today()) + "_LOG_" + str(logNum) + ".csv", "w+")
    logFile.write("Timestamp (Hour:Min:Sec), [L]Local Server Connection, [L]X, [L]Y, [L]Z, [L]Yaw, [L]Pitch, [L]Roll, [L]IsTracking, RangefinderDistance, [A]Altitude, \
        [A]Yaw, [A]Pitch, [A]Roll, InCh1, InCh2, InCh3, InCh4, ServoOut1, ServoOut2, ServoOut3, ServoOut4, ServoOut5, ServoOut6, ServoOut7, ServoOut8, FlightMode , Manual Control Mode \n")
    #logFile.close()
    
    try:    
        while True:
            #logFile = open(logDir + str(datetime.date.today()) + "_LOG_" + str(logNum) + ".txt", "a")
            now=datetime.datetime.now()
            
            logStr = ("" + \
            str(now.strftime('%H:%M:%S.%f')) + \
            "," + str(int(localConnect)) + \
            "," + str(x[0]) + \
            "," + str(y[0]) + \
            "," + str(z[0]) + \
            "," + str(yawAng[0]) + \
            "," + str(pitchAng[0]) + \
            "," + str(rollAng[0]) + \
            "," + str(isTracking[0]) + \
            "," + str(rangefinderDistance[0]) + \
            "," + str(autopilotAltitude[0]) )
            
            ##Add x,y,z from autopilot here. ax, ay, az
            ##also didn't get to rangefinder. Try something like @AutopilotCommunication.drone.vehicle.on_message('RANGEFINDER')
            
            logStr = logStr + (""+\
            "," + str(ayawAng[0]) + \
            "," + str(apitchAng[0]) + \
            "," + str(arollAng[0]) + \
            
            "," + str(rcInChannel[0]) + \
            "," + str(rcInChannel[1]) + \
            "," + str(rcInChannel[2]) + \
            "," + str(rcInChannel[3]) + \
            
            "," + str(servoOut[0]) + \
            "," + str(servoOut[1]) + \
            "," + str(servoOut[2]) + \
            "," + str(servoOut[3]) + \
            "," + str(servoOut[4]) + \
            "," + str(servoOut[5]) + \
            "," + str(servoOut[6]) + \
            "," + str(servoOut[7]) + \
            
            "," + str(flightMode[0]) +\
            "," + str(manualControlMode[0]) + "\n") 
            
            logFile.write(logStr)
            #print (logStr)
            time.sleep(.05)
    finally:
        logFile.close() 

#Main Thread call
if __name__ == '__main__':
    
    IMThread = threading.Thread(target = indoorMission)
    OMThread = threading.Thread(target = outdoorMission)
    
    threading.Thread(target = AutopilotCommunication).start()
    if LEDisEquipped == True:
        threading.Thread(target = LEDServerCommunication).start()
    if missionIsIndoor == True:
        threading.Thread(target = LocalServerCommunication).start() 
    threading.Thread(target = Logging).start() 
