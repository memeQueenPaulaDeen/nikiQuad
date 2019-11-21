import os
import datetime

def Logging():
    logNum = 1
    while os.path.isfile("/home/pi/LogFiles/" + str(datetime.date.today()) + "_LOG_" + str(logNum) + ".txt"):
        logNum += 1
        
    logFile = open("/home/pi/LogFiles/" + str(datetime.date.today()) + "_LOG_" + str(logNum) + ".txt", "a")
    logFile.write("LOG TEST")
    logFile.close()
    
    
Logging()
