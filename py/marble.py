#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 23 22:19:29 2020

Linux information:

To install library on linux:
python -m pip install pyserial

If you get a permissions error then run this command
in a terminal

sudo chmod a+rw /dev/ttyACM0

If you are not sure if the device is sending data, then run
these commands in a terminal 

sudo chmod a+rw /dev/ttyACM0
cat /dev/ttyACM0   

If you want to reprogramme the arduino you will need to close the python console to 
free up the serial port

@author: Timothy Drysdale (timothy.d.drysdale@gmail.com)
"""

import json
import serial

verbose = False #show messages from arduino

def student_function(times,finished):
    """
    This is an example of a student function. 
    The idea would be to give the students this code
    but with no content in this function (don't alter 
    the main code though)
    
    """
    
    lane1 = times[1]-times[0]
    lane2 = times[2]-times[0]
    lane3 = times[3]-times[0]
    lane4 = times[4]-times[0]
    
    lane_list = ["lane 1","lane 2","lane 3","lane 4"]
    lane_times = [lane1,lane2,lane3,lane4]
    
    sorted_lanes = [x for _,x in sorted(zip(lane_times,lane_list))]
    
    lane_times.sort()
    
    positions = ["first","second","third","fourth"]
    print("RACE REPORT")
    for lane, pos, ms in zip(sorted_lanes, positions, lane_times):
        print(pos,":",lane," in ", ms ,"ms")
    
    
# DO NOT ALTER CODE BELOW THIS LINE!!
        
if __name__ == "__main__":
    
    # this is the serial port name on linux - will be different on windows
    # see arduino documentation for information on what it will be
    ser = serial.Serial('/dev/ttyACM0',115200,timeout=10)  # open serial port
    
    while True:
    
        line = ser.readline()
    
        try:
            
            obj = json.loads(line)
            
            if verbose:
                print(obj)
        
            if obj["msg"] == "report":
                times = obj["raw_times"]
                finished = obj["finished"]
                
                student_function(times,finished)
                            
        except:
            continue #ignore any blank lines from serial timeout etc
        
           
    ser.close()             # close port (currently this is unreachable code)
