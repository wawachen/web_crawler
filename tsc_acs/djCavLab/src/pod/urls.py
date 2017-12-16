'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

from django.conf.urls import url

from . import views

urlpatterns = [
    url(r'^pointCloud/(?P<template>[0-1])$', views.pointCloud, name='pointCloud'),
    url(r'^pointCloud$', views.pointCloud, name='pointCloud'),
    url(r'^summary', views.summary, name='summary'),
]