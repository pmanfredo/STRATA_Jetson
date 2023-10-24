#!/usr/bin/env python

import serial
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, Float64MultiArray
import struct, array

SERIAL_PORT = '/dev/ttyTHS1'

# COMMANDS
SET_WHEEL_SPEED = bytes(b'\x01') # 4 double array of speeds (rpm)
STOP_WHEELS = bytes(b'\x02') # No args
GET_WHEEL_SPEED = bytes(b'\x03') # Returns 4 double array of speeds (rpm)
GET_WHEEL_POSITION = bytes(b'\x04') # Returns 4 double array of position (revol
SET_TURN_ANGLE = bytes(b'\x07') # int32 angle in degrees

RETURN_WHEEL_SPEED = bytes(b'\x05')
RETURN_WHEEL_POSITION = bytes(b'\x06')

target_speed = 0
target_turn = 0

def speedCallback(data):
    global target_speed
    target_speed = data.data
    
def turnCallback(data):
    global target_turn
    target_turn = data.data

if __name__ == "__main__":
    # Initialize node
    rospy.init_node('wheelControl')
    rospy.loginfo("Running")
    
    rospy.Subscriber("wheelControl/target_speed", Float32, speedCallback)
    rospy.Subscriber("wheelControl/turn_angle", Float32, turnCallback)
    
    positionPub = rospy.Publisher('wheelControl/position', Float64MultiArray, queue_size=1000)
    speedPub = rospy.Publisher('wheelControl/speed', Float64MultiArray, queue_size=1000)
    
    ser = serial.Serial(SERIAL_PORT, 115200)
    
    ser.write(0)
    
    rate = rospy.Rate(10)
    
    last_wheel_speed = [0]*4
    last_turn = 0
    while not rospy.is_shutdown():
        ser.write(GET_WHEEL_SPEED)
        ser.write(GET_WHEEL_POSITION)
        
        wheel_speeds = [0]*4
        wheel_position = [0]*4
        
        while ser.in_waiting:
            cmd = ser.read(1)
            
            if(cmd == RETURN_WHEEL_SPEED):
                data = ser.read(8*4)
                
                wheel_speeds = list(array.array('d', data))
                    
                data_to_send = Float64MultiArray()
                data_to_send.data = wheel_speeds
                speedPub.publish(data_to_send)
                    
            if(cmd == RETURN_WHEEL_POSITION):
                data = ser.read(8*4)
                
                wheel_position = list(array.array('d', data))
                    
                data_to_send.data = wheel_position
                data_to_send = Float64MultiArray()
                positionPub.publish(data_to_send)

                    
        
        if (wheel_speeds != last_wheel_speed) and (target_turn != last_turn):
            continue
        
        last_wheel_speed = wheel_speeds
        last_turn = target_turn
        
        wheel_speeds = [target_speed]*4
        
        # Set turn angle
        data = bytearray(SET_TURN_ANGLE)
        data.extend(struct.pack("i", int(target_turn)))
        ser.write(data)
        
        # Set wheel speed
        if target_speed == 0:
            ser.write(STOP_WHEELS)
        else:
            data = bytearray(SET_WHEEL_SPEED)
            for i in wheel_speeds:
                data.extend(struct.pack("d", i))
                
            ser.write(data)
            
        rate.sleep()
        
    ser.close()