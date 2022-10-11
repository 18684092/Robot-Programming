#!/usr/bin/env python3

##########################################################
# Author        :   Andy Perrett (18684092)              #
# Email         :   andy@wired-wrong.co.uk               #
# Date          :   22nd September 2022                  #
# Course        :   MSc RAS                              #
# Module        :   Robot Programming                    #
# Version       :   1.0                                  #
# Description   :   Workshop for ROS subscriber running  #
#               :   and making connection robust         #
##########################################################

# ROS imports
import rospy
from std_msgs.msg import String

# Other imports
import time
import socket   # Used to "ping" ROS master
import os       # Just for environ ROS_MASTER_URI

# Imports for restarting
import sys
import psutil
import logging

# A little trouble with urlparse
# TODO cleanup maybe? 
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

# Globals
TIMEOUT = 10
LAST_MESSAGE = time.time()

##############
# get_master #
##############
def get_master():
    # ROS master URL should be set within .bashrc, else default
    try:
        uri = urlparse(os.getenv('ROS_MASTER_URI'))
        (host, port) = uri.netloc.split(':')
        return (host, int(port))
    except:
        return ('127.0.0.1', 11311)

###############
# ping_master #
###############
# Taken from
# https://answers.ros.org/question/301135/how-to-check-if-network-connection-or-remote-ros-master-is-down/
def ping_master(master):
    # try to ping the master IP by connecting successfully within timeout
    try:
        socket.setdefaulttimeout(1)
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        soc.connect(master)
    except: 
        return False
    else:
        soc.close()
        return True

############
# callback #
############
def callback(data):
    global LAST_MESSAGE
    LAST_MESSAGE = time.time()
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

################
# restart node #
################
# Taken from
# https://stackoverflow.com/questions/11329917/restart-python-script-from-within-itself
def restart_node():
    """Restarts the current program, with file objects and descriptors
       cleanup
    """

    try:
        p = psutil.Process(os.getpid())
        for handler in p.get_open_files() + p.connections():
            os.close(handler.fd)
    except Exception as e:
        logging.error(e)

    python = sys.executable
    os.execl(python, python, *sys.argv)

########
# main #
########
def main():

    topic = 'free_mem'

    # Init will block, waiting for master, before continuing...
    rospy.init_node('listener')
    rate = rospy.Rate(10)
    rospy.Subscriber(topic, String, callback)

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
        
        master = get_master()
        connected = ping_master(master)

        # If master can be connected to AND we are known to have GONE offline...
        if connected and offline:
            
            print("{}: {} - ROS master is online".format(time.strftime("%d.%m.%Y-%H:%M:%S"),rospy.get_name()))
            offline = False 
            rospy.init_node('listener')
            rospy.Subscriber(topic, String, callback)
            rospy.loginfo("%s: will listen on topic \\%s", rospy.get_name(), topic)

        # if master can NO LONGER be connected to, and we WERE online...
        elif not connected and not offline:

            print("{}: {} - ROS master went offine".format(time.strftime("%d.%m.%Y-%H:%M:%S"),rospy.get_name())) 
            offline = True

        # if connected AND we haven't gone down (normal state)... 
        elif connected and not offline:
            # Do stuff when not in callback

            # A simple timeout - probably publisher restarted
            if time.time() > LAST_MESSAGE + TIMEOUT:
                rospy.loginfo("{}: {} - Topic: {} timedout - restarting node".format(time.strftime("%d.%m.%Y-%H:%M:%S"),rospy.get_name(), topic))
                restart_node()

    rospy.loginfo("%s: is shutting down deliberately, probably ctrl-c", rospy.get_name())

#############
# Call main #
#############
if __name__ == "__main__":
    main()