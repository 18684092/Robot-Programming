#!/usr/bin/env python3

##########################################################
# Author        :   Andy Perrett (18684092)              #
# Email         :   andy@wired-wrong.co.uk               #
# Date          :   22nd September 2022                  #
# Course        :   MSc RAS                              #
# Module        :   Robot Programming                    #
# Version       :   1.0                                  #
# Description   :   Workshop for ROS publisher running   #
#               :   and making connection robust         #
##########################################################

# ROS imports
import rospy
from std_msgs.msg import String

# Other imports
import json
import psutil   # Get memory details
import time     # We add an epoch but its not necessary (NOTE ?)
import socket   # Used to "ping" ROS master
import os       # Just for environ ROS_MASTER_URI

# A little trouble with urlparse, "exception:" is the correct one but
# earlier pythons (or Windows?) may require the "try:"
# TODO cleanup maybe? 
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

# Ideas taken from
# https://www.youtube.com/watch?v=Q5y-3aZdzfQ (services)

###########
# Sensors #
###########
class Sensors:
    ''' Returns sensor info as a serialised dictionary '''
    
    ############
    # __init__ #
    ############
    def __init__(self, field='free'):
        self.field = field
        rospy.loginfo("%s initiated", rospy.get_name())
        self.master = self.get_master()

    ##############
    # get_master #
    ##############
    def get_master(self):
        # ROS master URL should be set within .bashrc, else default
        try:
            uri = urlparse(os.getenv('ROS_MASTER_URI'))
            (host, port) = uri.netloc.split(':')
            return (host, int(port))
        except:
            return ('127.0.0.1', 11311)

    ######################
    # get_memory details #
    ######################
    def get_memory_details(self):
        ''' Returns details about memory usage '''

        raw = psutil.virtual_memory()

        memory = {
            "total": raw.total,
            "available": raw.available,
            "percent": raw.percent,
            "used": raw.used,
            "free": raw.free,
            "active": raw.active,
            "inactive": raw.inactive,
            "buffers": raw.buffers,
            "cached": raw.cached,
            "shared": raw.shared
        }
        # NOTE
        # Some oddity - psutil for python2.7 doesn't have slab
        # python 3 psutil does. BUT I have one Ubuntu 18.04 PC that
        # does have Python2.7 and slab and one that doesn't
        try:
            memory['slab'] = raw.slab
        except AttributeError:
            pass

        # If specific request - send the correct field otherwise the entire dictionary is published
        if self.field in memory:
            memory = memory[self.field]
        
        memory = json.dumps(memory)
        return memory

    ##############
    # pub_memory #
    ###############
    def pub_memory(self):
        memory = String(self.get_memory_details())
        return memory


    ###############
    # ping_master #
    ###############
    # Taken from
    # https://answers.ros.org/question/301135/how-to-check-if-network-connection-or-remote-ros-master-is-down/
    def ping_master(self):
        # try to ping the master IP by connecting successfully within timeout
        try:
            socket.setdefaulttimeout(1)
            soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            soc.connect(self.master)
        except: 
            return False
        else:
            soc.close()
            return True


########
# main #
########
def main():
    
    # We can select the psutil memory field
    # set field to 'all' to publish all memory fields or e.g. field = 'free' to publish
    # just a single memory field
    field = 'free'
    field_topic = field + "_mem"

    # Init will block, waiting for master, before continuing...
    rospy.init_node("sensor_pub")

    # We are connected to master, setup publisher
    rate = rospy.Rate(10)
    monitor = Sensors(field)
    pub = rospy.Publisher(field_topic, String, queue_size=10)
    rospy.loginfo("%s: will publish memory field: %s on topic \\%s", rospy.get_name(), field, field_topic)


    # If master goes down, detect, wait for master to come back, and re-init
    # Taken from (and modified 'bigly' to suit me)
    # https://answers.ros.org/question/301135/how-to-check-if-network-connection-or-remote-ros-master-is-down/
    offline = False

    # Loop testing that we can STILL connect to ROS master
    while not rospy.is_shutdown():

        try:
            rate.sleep()
        except rospy.ROSInterruptException: 
            break
        
        connected = monitor.ping_master()

        # If master can be connected to AND we are known to have GONE offline...
        if connected and offline:
            
            print("{}: {} - ROS master is online".format(time.strftime("%d.%m.%Y-%H:%M:%S"),rospy.get_name()))
            offline = False 
            rospy.init_node("sensor_pub")
            pub = rospy.Publisher(field_topic, String, queue_size=10)
            rospy.loginfo("%s: will publish memory field: %s on topic \\%s", rospy.get_name(), field, field_topic)

        # if master can NO LONGER be connected to, and we WERE online...
        elif not connected and not offline:

            print("{}: {} - ROS master went offine".format(time.strftime("%d.%m.%Y-%H:%M:%S"),rospy.get_name())) 
            offline = True

        # if connected AND we haven't gone down (normal state)... 
        elif connected and not offline:
            message = monitor.pub_memory()
            pub.publish(message)

    rospy.loginfo("%s: is shutting down deliberately, probably ctrl-c", rospy.get_name())

#############
# Call main #
#############
if __name__ == "__main__":
    main()

