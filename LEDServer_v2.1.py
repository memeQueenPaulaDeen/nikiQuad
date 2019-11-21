#!/usr/bin/python
from bootstrap import *
import time
from threading import Thread
import socket


#############  LED ID NUMBER REFERANCE  ####################
#-in-out--in-out
#-(0-5)-(18,23)
#----\\-//-----
#-----|O|------ DRONE UP=FORWARD BIRDS-EYE-VIEW
#----//-\\-----
#(6-11)-(12,17)
#in-out--out-in
############################################################

#NESSESSARY VARIABLES


venableLED = True
vbatState = 100
vrcState = True
vautopilot = False
vgpsState = 0
vindoorOutdoor = True
vautoland = False


blink = False #Global for value for blinking functions

led = LEDStrip(24) #LED pin call -> easy-to-type variable


########################################################
#COLORS

red = Color(255,0,0)
yellow = Color(255,255,0)
purple = Color(137,0,204)
green = Color(0,255,0)
blue = Color(0,0,255)
orange = Color(255, 120, 0)
white = Color(255,255,255)
pink = Color(244,66,232)
off = Color(0,0,0)

########################################################
#FUNCTIONS


#Indicates level of battery
def BatteryMonitor (state,rcConnect, indoorOutdoor, blink):
		
    if rcConnect: #Connected to RC
        color = blue
    else:
        color = red

    if state < 36 and blink: #For LEDs to blink under 10%
        led.fill(color,6,17)
    elif state < 36:
        led.fill(off,6,17)
		
		
    if True: ###indoorOutdoor: #IF INDOORS
        if state > 87: #Threshholds of battery (%)
            led.fill(color,6,11)
            led.fill(color,12,17)
        elif state > 74:
            led.fill(color,6,10)
            led.fill(off,11,11) #Fills empty part with off LEDs
            led.fill(color,13,17)
            led.fill(off,12,12) 
        elif state > 62:
            led.fill(color,6,9)
            led.fill(off,10,11)
            led.fill(color,14,17)
            led.fill(off,12,13)
        elif state > 49:
            led.fill(color,6,8)
            led.fill(off,9,11)
            led.fill(color,15,17)
            led.fill(off,12,14) 
        elif state >= 36:
            led.fill(color,6,7)
            led.fill(off,8,11)
            led.fill(color,16,17)
            led.fill(off,12,15)
        #elif state >= 10:
        #    led.fill(color,6,6) #only 1 led
        #    led.fill(off,7,11)
        #    led.fill(color,17,17)
        #    led.fill(off,12,16)	

    else: #Outdoors
        if state > 50: #Threshholds of battery (%)
            led.fill(color,6,11)
            led.fill(color,12,17)
        elif state > 25:
            led.fill(color,6,8)
            led.fill(off,9,11)
            led.fill(color,15,17)
            led.fill(off,12,14)
        elif state >= 10:
            led.fill(color,6,6)
            led.fill(off,7,11)
            led.fill(color,17,17)
            led.fill(off,12,16)




#Indicates autopilot communication (blinking purple)
def Autopiolet (state,blink):
#State true autopiolet on.
    if not state and blink:
        led.fill(purple,0,23)
    elif not state:
        led.fill(off,0,23)
    else:
        return 0
        
        
#Indicates GPS state based on value number
def GpsState (stateVal, blink):
    if stateVal == 0:
        led.fill(off,0,5)
        led.fill(off,18,23)
    elif stateVal == 1:
        led.fill(purple,0,5)
        led.fill(purple,18,23)
    elif stateVal == 2:#
        led.fill(pink,0,5)
        led.fill(pink,18,23)
    elif stateVal == 3:#
        led.fill(yellow,0,5)
        led.fill(yellow,18,23)
    elif stateVal == 4:
        led.fill(green,0,5)
        led.fill(green,18,23)    
    elif stateVal == 5:#
        led.fill(green,0,5)
        led.fill(green,18,23)
    elif stateVal == 6:
        led.fill(yellow,0,5)
        led.fill(yellow,18,23)
        
    if blink and stateVal in (2,3,5): #Blink condition for GPS states
        led.fill(off,0,5)
        led.fill(off,18,23)	

#Indicates land mode = Autoland ; Front LEDs white
def AutoLand (state):
	if state:
		led.fill(white,0,5)
		led.fill(white,18,23)


#If LED disabled, turn all LEDs off	
def EnableLED (enabled):
	if not enabled:
		led.fill(off,0,23)
	
	
######################################################
#MEGA FUNCTION!


def LED(enableLED,batState, rcState, autopiolet, gpsState, indoorOutdoor, autoland):	

###########################################
#VARIABLES:

##enableLED -- allow LEDs to color. True is all LEDs show, off is LEDs off

##batState -- (int) Battery state value 0-100

##rcState -- (boolian) if RC Connected. true if connected

##autopiolet -- (boolian) if autopiolet is on

##gpsState -- (int) GPS state value 0-6.
###(0) Initial - VALUE 0 : LEDS OFF
###(1) Local Server Connected, isTracking, - VALUE 1 : SOLID PURPLE
###(2) Local Server Connected, no isTracking - VALUE 2 : BLINKING PINK
###(3) Local Server Disconnected, Outdoor GPS Disabled - VALUE 3 : BLINKING YELLOW
###(4) Local Server Disconnected, Outdoor GPS Enabled, HDOPs < .5 - VALUE 4 : SOLID GREEN
###(5) Local Server Disconnected, Outdoor GPS Enabled, HDOPs < 1 - VALUE 5 : BLINKING GREEN
###(6) Local Server Disconnected, Outdoor GPS Enabled, HDOPs > 1 - VALUE 6 : SOLID YELLOW

##indoorOutdoor -- (boolian) is the drone indoors or outdoors, true if indoors

##autoland -- (boolian) if autoland is on

############################################


#STATE LASTS FOR 1 SECOND. FUNCTION CALL WITHOUT CONDITIONAL AND LOOP UNLIKELY
	
    global venableLED
    global vbatState
    global vrcState
    global vautopilot
    global vgpsState
    global vindoorOutdoor
    global vautoland
            
	
    while True:
		
        enableLED = venableLED
        batState = vbatState
        rcState = vrcState
        autopilot = vautopilot
        gpsState = vgpsState
        indoorOutdoor = vindoorOutdoor
        autoland = vautoland
		
        global blink
        if blink:
            blink = False
        else:
            blink = True
        
        led.fill(yellow,0,23) #Test. If all LED alerts are bypassed, LEDs yellow.
    
        #All smaller functions called here
        GpsState(gpsState, blink)
        BatteryMonitor(batState, rcState, indoorOutdoor, blink)
        AutoLand(autoland)
        EnableLED(enableLED)
        Autopiolet(autopilot, blink)
    
    
        led.update() #Updates colors of LEDs
        time.sleep(.5)
    
    
###################################################################

#Creates server and connects to client IndoorMission
def NetworkConnect():
	
    global venableLED
    global vbatState
    global vrcState
    global vautopilot
    global vgpsState
    global vindoorOutdoor
    global vautoland
     
    try:        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "0.0.0.0"
        port = 16884
        s.bind((host,port))
    except Exception as e:
        WriteLog(str(e))
        
    while True:
        try:
            
            s.listen(5)   
            print("Waiting for a client to connect...")             
            c, addr = s.accept()     #establish connection with client.
            print 'Got connection from', addr
            while 1:
                data = c.recv(128)
                #print data
                arrayList = data.split(',')
                print arrayList


                ### Assign received data to global variables ### 
                if arrayList[0]=='1':
                    venableLED = True
                    print "enabled LED"
                elif arrayList[0]=='0':
                    print "disabled LED"
                    venableLED = False
                #print arrayList[1]
                vbatState = int(arrayList[1])
                #print vbatState

                if arrayList[2]=='1': vrcState = True
                elif arrayList[2]=='0': vrcState = False
                #print vrcState
                
                if arrayList[3]=='1': vautopilot = True
                elif arrayList[3]=='0': vautopilot = False
                
                #print vautopilot
                vgpsState = int(arrayList[4])
	            
                if arrayList[5]=='1': vindoorOutdoor = True
                elif arrayList[5]=='0': vindoorOutdoor = False
	            
                if arrayList[6]=='1': vautoland = True
                elif arrayList[6]=='0': vautoland = False
	            ################################################
                time.sleep(.17)
                
                if not data:
                     
                    print("NO DATA-----------")
                    c.close()
                    #s.close()
                    vautopilot = False
                    break
            

        except Exception, e: 
            print "[-] " + str(e)
            WriteLog(str(e))
            c.close()
            #s.close()
            vautopilot = False
            time.sleep(1)
            continue

        finally:
            vautopilot = False
            c.close()
            s.close()
        
   # s.shutdown(SHUT_RDWR)

def WriteLog(msg):
        fileVar = open("/home/pi/DroneProject/LEDLog.txt", "r+") #Write exception to file LEDLog.txt
        fileVar.write(msg)
        fileVar.close()

if True: #while True:
    if __name__ == '__main__':
        Thread(target = LED, args = (venableLED, vbatState, vrcState, vautopilot, vgpsState, vindoorOutdoor, vautoland)).start()
        Thread(target = NetworkConnect).start()

