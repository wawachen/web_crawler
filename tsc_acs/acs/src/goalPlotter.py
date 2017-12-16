#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

"""
Python node to plot live gps location and the goal against the route.
"""

# Import required Python code.
import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Button

import threading

# Import custom message data.
from sensor_msgs.msg import NavSatFix
from oxts.msg import BatchB
from acs.msg import Pose2DStamped
from geometry_msgs.msg._Pose2D import Pose2D
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg import Header


pub = rospy.Publisher('/acs/path_reset', Header)

class Index(object):
    ind = 0

    def zoomIn(self, event):
        global zoom
#         print zoom
        if zoom:
            zoom = False
            print "Zooming Out"

        else:
            zoom = True
            print "Zooming In"

        zoom = zoom
#         print zoom
#         self.ind += 1
#         i = self.ind % len(freqs)
#         ydata = np.sin(2*np.pi*freqs[i]*t)
#         l.set_ydata(ydata)
#         plt.draw()

    def resetGoal(self, event):
        print "resetting goal"
        pub.publish(Header())
#         self.ind -= 1
#         i = self.ind % len(freqs)
#         ydata = np.sin(2*np.pi*freqs[i]*t)
#         l.set_ydata(ydata)
#         plt.draw()

def latLngTOxy(data):
    '''
    convert gps location to xy
    '''
    
    originTemp = rospy.get_param("/acs/origin")
    origin = originTemp[0:2]
    print origin
    radiusEarth = 6371000  
    
    x = radiusEarth * np.cos(data[:,0] * np.pi / 180.0) * (data[:,1] - origin[1]) * np.pi / 180.0
    y = radiusEarth* (data[:,0] - origin[0]) * np.pi/180.0

    xy = np.array([x,y])
    return np.transpose(xy)

def callbackAcsGoal(data):
    global xGoal
    global yGoal
    global thetaGoal
    global newData
    global xyGoalLine
#     lock.acquire()

    xGoal = data.pose.x
    yGoal = data.pose.y
    thetaGoal = data.pose.theta
    
    xyGoalLine = np.zeros([2,2])
    xyGoalLine[0,0] = data.pose.x
    xyGoalLine[0,1] = data.pose.y
    xyGoalLine[1,0] = data.pose.x + 2*np.cos(data.pose.theta)
    xyGoalLine[1,1] = data.pose.y + 2*np.sin(data.pose.theta)

    newData = True
#     lock.release()

def callbackAcsLocation(data):
    global xLocation
    global yLocation
    global thetaLocation
    global newData
    global xyLine
#     global yLimits
#     lock.acquire()

    xLocation = data.pose.x
    yLocation = data.pose.y
    thetaLocation = data.pose.theta

    xyLine = np.zeros([2,2])
    xyLine[0,0] = data.pose.x
    xyLine[0,1] = data.pose.y
    xyLine[1,0] = data.pose.x + 2*np.cos(data.pose.theta)
    xyLine[1,1] = data.pose.y + 2*np.sin(data.pose.theta)

#     yLimits = np.zeros([2,2])
#     yLimits = 2*np.sin(thetaLocation)+[-4,4]

    newData = True
#     lock.release()

def listener():
    '''
    Main function.
    '''
#     global H
#     global x
#     global y
    global xGoal
    global yGoal
    global thetaGoal
    global xyGoalLine
# 
    global xLocation
    global yLocation
    global thetaLocation
    global xyLine
    global zoom
#     global yLimits
# 
# 
    global newData
#     global route

    xGoal = 0
    yGoal = 0
    thetaGoal = 0
    xyGoalLine = np.zeros([2,2])
    xLocation = 0
    yLocation = 0
    thetaLocation = 0
    xyLine = np.zeros([2,2])
#     yLimits = [-4,4]
    zoom = False
    
    plt.ion()

#     fig1 = plt.fig
#     map = mpimg.imread('/home/ujjar/B4 Test Area.png')
#     mapExtent = [-0.76738, -0.76482, 52.03337, 52.03503]
#     axRoute = plt.axis([0, 0, 0.1, 10])
    axRoute = plt.axes([0.05 , 0.1, 0.9, 0.8])
#     axRoute = plt.axes([min(routeXY[:,0]) , max(routeXY[:,0]), min(routeXY[:,1]) , max(routeXY[:,1])])
    hGoal = plt.plot(0,0,'+g', markersize=10)
    hGoalLine = plt.plot(0,0,'-g', markersize=0)
    hLocation = plt.plot(0,0,'*r',  markersize=10)
    hLine = plt.plot(0,0,'-r',  markersize=0)
    hRoute = plt.plot(routeXY[0:-1:10,0],routeXY[0:-1:10,1],'--k')
    plt.grid(True)
    plt.axis("equal")

   
   
    callback = Index()  
    axZoomIn = plt.axes([0.65, 0.01, 0.15, 0.075])
    axResetGoal = plt.axes([0.81, 0.01, 0.1, 0.075])

    plt.sca(axRoute)
#     axRoute = plt.axes([0,0,0,0])

#     plt.imshow(map, extent=mapExtent)
#     plt.axis([-60, 20, -60, 60])
#     plt.axis("equal")
#     plt.axis([-60, 20, -60, 60])

    rospy.Subscriber('/acs/goal', Pose2DStamped, callbackAcsGoal)
    rospy.Subscriber('/acs/location', Pose2DStamped, callbackAcsLocation)

    while True:
        if newData:
#             lock.acquire()
            hGoal[0].set_ydata(yGoal)
            hGoal[0].set_xdata(xGoal)
            hGoalLine[0].set_ydata(xyGoalLine[:,1])
            hGoalLine[0].set_xdata(xyGoalLine[:,0])

            hLocation[0].set_ydata(yLocation)
            hLocation[0].set_xdata(xLocation)
            hLine[0].set_ydata(xyLine[:,1])
            hLine[0].set_xdata(xyLine[:,0])
#             bZoomIn.on_clicked(callback.zoomIn)

            if zoom:
                plt.axis([xLocation-4 + 2* np.cos(thetaLocation), xLocation+4  + 2* np.cos(thetaLocation), 
                      yLocation-4 + 2* np.sin(thetaLocation), yLocation+4 + 2* np.sin(thetaLocation)])
            else:
                plt.axis([min(routeXY[:,0]) , max(routeXY[:,0]), min(routeXY[:,1]) , max(routeXY[:,1])])
#                 axRoute.autoscale()
            
#             plt.axis([xLocation-4, xLocation+4, yLocation-4, yLocation+4])
            bZoomIn = Button(axResetGoal, 'Zoom')
            bZoomIn.on_clicked(callback.zoomIn)
            bResetGoal = Button(axZoomIn, 'Reset Goal', )
            bResetGoal.on_clicked(callback.resetGoal)
            
            plt.pause(0.001)
#             lock.release()


            
# Main function.
if __name__ == '__main__':
#     global x
#     global y
#     global xGoal
#     global yGoal
#     global newData
#     global lock
#     global route
    x = []
    y = []
    newData = False
#     route_file = ''

    routeFile = raw_input("Enter route filename.\n")
#     routeFile = rospy.get_param('routeFile','firstA.csv')
    print routeFile
    route = np.genfromtxt(os.environ.get('HOME')+'/catkin_acs/src/tsc_acs/routes/'+routeFile+'.csv', delimiter=',')
#     route = np.genfromtxt(os.environ.get('HOME')+'/catkin_acs/src/tsc_acs/routes/routeB.csv', delimiter=',')
    routeXY = latLngTOxy(route)
#     lock = threading.Lock()
    # Initialize the node and name it.
    rospy.init_node('goalPlotter')
    # Go to the main loop.
    listener()
