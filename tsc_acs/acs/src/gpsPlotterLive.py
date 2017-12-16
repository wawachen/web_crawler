#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

"""
Example Python node to plot on a specific topic.
"""

# Import required Python code.
import rospy
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import threading

# Import custom message data.
from sensor_msgs.msg import NavSatFix

def callback(data):
    '''
    Callback function for the subscriber.
    '''
    global x
    global y
    global newData
    
    lock.acquire()
    x.append(data.longitude)
    y.append(data.latitude)
    newData = True
    lock.release()

def listener():
    '''
    Main function.
    '''
    global H
    global x
    global y
    global newData
    
    plt.ion()
#     map2 = mpimg.imread('/home/ujjar/MK9.png')
#     map2Extent = [-0.77438, -0.7629, 52.03188, 52.04123]
    map = mpimg.imread('/home/ujjar/B4 Test Area.png')
    mapExtent = [-0.76738, -0.76482, 52.03337, 52.03503]

    H = plt.plot(0,0,'.b-')
    plt.imshow(map, extent=mapExtent)
#     plt.imshow(map2, extent=map2Extent)

    plt.axis([-0.766497, -0.76482, 52.03337, 52.034539]) 
#     plt.axis([-0.763, -0.770, 52.02, 52.04])
#     plt.grid(True)
    rospy.Subscriber('/gps/fix', NavSatFix, callback)
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    while True:
        if newData:
            lock.acquire()
            H[0].set_ydata(y)
            H[0].set_xdata(x)
            newData = False
            plt.pause(0.001)
            lock.release()
            
# Main function.
if __name__ == '__main__':
    global x
    global y
    global newData
    global lock
    x = []
    y = []
    newData = False
    lock = threading.Lock()
    # Initialize the node and name it.
    rospy.init_node('pyplotter')
    # Go to the main loop.
    listener()
