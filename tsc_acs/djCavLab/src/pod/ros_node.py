'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

import rospy
from threading import Thread
import time
from sensor_msgs.msg import PointCloud2
'''from lutz.msg import UICode
'''
class rosNode:
    class __node(Thread):
        def __init__(self):
            Thread.__init__(self)
            
            rospy.init_node('djCavLab', disable_signals=True)
            
            self.rate = rospy.Rate(4)
            self.pointCloud = None
            self.uiCode = None
            self.subPC = rospy.Subscriber("/ibeo/point_cloud", PointCloud2, pointCloudCallback, [self])
            '''            self.subUI = rospy.Subscriber("/ui_code", UICode, uiCodeCallback, [self])
'''            
            self.start()
            
        def run(self):
            print ("Starting Node Thread ")
            while not rospy.is_shutdown():
                self.rate.sleep()
            
    node = None
    
    def __init__(self):
        if rosNode.node == None:
            rosNode.node = rosNode.__node()
    
    def getPointCloud(self):
        return rosNode.node.pointCloud
    
    def getUICode(self):
        return rosNode.node.uiCode
                
def pointCloudCallback(msg, args):
    node = args[0]
    node.pointCloud = msg
                
def uiCodeCallback(msg, args):
    node = args[0]
    node.uiCode = msg

rosNode()