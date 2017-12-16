'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

import rospy
from std_msgs.msg import Int32
from threading import Thread
import time

class rosNode:
    class __node(Thread):
        def __init__(self):
            Thread.__init__(self)
            
            rospy.init_node('djCavLab', disable_signals=True)
            
            self.rate = rospy.Rate(1)
            self.count = 'Not received'
            self.sub = rospy.Subscriber("count", Int32, rxCallback, self)
            self.start()
            
        def run(self):
            print ("Starting Node Thread ")
            while not rospy.is_shutdown():
                self.rate.sleep()
            
    node = None
    
    def __init__(self):
        if rosNode.node == None:
            rosNode.node = rosNode.__node()
    
    def value(self):
        return rosNode.node.count

                
def rxCallback(msg, node):
    node.count = msg.data
            