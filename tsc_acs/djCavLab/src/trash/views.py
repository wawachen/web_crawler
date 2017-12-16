'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

from django.shortcuts import render
from django.http import HttpResponse
import trash.ros_node

def hello(request):
    return HttpResponse("Hello World!")

def count(request):
    ros = trash.ros_node.rosNode()
    
    return HttpResponse("COUNT: " + str(ros.value()))