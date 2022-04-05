#!/usr/bin/env python3
import serial
import rospy
import datetime
import utm
from gps_driver.msg import gps

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 4800
b = []

global latitude_deg
msg = gps()
def talker():
    pub = rospy.Publisher('gps_message', gps, queue_size=10)
    rospy.init_node('gps_talker', anonymous=True)
    r = rospy.Rate(10)
       


    while not rospy.is_shutdown():
            
        a = str(ser.readline())
        b = a.split("b'")
        c = b[1].split(",")
        if c[0] == "$GPGGA"
            print(c)
            latitude_gps = float(c[2])
            latitude_mins = latitude_gps%100
            latitude_degree = int(latitude_gps/100)
            latitude = latitude_degree + (latitude_mins/60)

            longitude_gps = float(c[4])
            longitude_mins = longitude_gps%100
            longitude_degree = int(longitude_gps/100)
            longitude1 = longitude_degree + (longitude_mins/60)
            longitude = -1*longitude1

        
            u = utm.from_latlon(latitude, longitude)
            print(u)
            altitude = float(c[9])
            
            msg.header = c[0]
            msg.latitude = float(latitude)
            msg.longitude = float(longitude)
            msg.altitude = float(altitude)
            msg.utm_easting = float(u[0])
            msg.utm_northing = float(u[1])
            msg.zone = int(u[2])
            msg.letter = u[3]
            rospy.loginfo(msg)
            pub.publish(msg)
            r.sleep()


if __name__ == '__main__':
    # try:
    talker()
    # except rospy.RosInterruptException:
    #     pass
