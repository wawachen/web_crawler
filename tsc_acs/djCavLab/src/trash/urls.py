'''*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
#H'''

from django.conf.urls import url

from . import views

urlpatterns = [
    url(r'^hello$', views.hello, name='hello'),
    url(r'^count', views.count, name='count'),
]