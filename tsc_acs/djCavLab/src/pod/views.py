'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

from django.shortcuts import render
from django.http import HttpResponse
import pod.ros_node

def summary(request):
    ros = pod.ros_node.rosNode()
    
    uiCode = ros.getUICode()
    return HttpResponse(str(uiCode))

def pointCloud(request, template=1):
    ros = pod.ros_node.rosNode()
    
    pointCloud = ros.getPointCloud()
    
    if template != "0":
        return render(request, 'pod/pointCloud.html', {'pointCloud': pointCloud})
    else:
        return HttpResponse(str(pointCloud)) 