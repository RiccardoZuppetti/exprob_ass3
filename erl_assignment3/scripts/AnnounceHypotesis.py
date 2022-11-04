#! /usr/bin/env python

##  \package erl_assignment3
#   
#   \file AnnounceHypotesis.py
#   \brief ROS node used for simulating the robot announcement
#   \author Riccardo Zuppetti
#   \version 1.0
#   \date 22/08/2022
#   
#   \details
#   
#   Subscribes to: <BR>
#       None
# 
#   Publishes to: <BR>
#       None
# 
#   Client: <BR>
#       None
#   
#   Service: <BR>
#       /announce_service
#
#   Action Client:
#       /move_base
#
#   Description: <BR>
#
#       Given an hypothesis, this is printed on the screen.


import rospy
import random
from erl_assignment3.srv import Announcement, AnnouncementResponse
import actionlib

from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseAction, MoveBaseGoal

##
#   \brief Callback function
#   \param req : data retrieved on the /announce_service topic
#   \return : true
#
#   This function retrieves the request field of the Announcement message. Inside the custom message is present the istances of classes PERSON, PLACE, WEAPON
#   corresponding to the hypothesis to announce. The robot firstly reach the centre of the arena, then announce the hypothesis and finally returns to the initial location.
#

def announce_clbk(req):

    Room_x = [-4,-4,-4,5,5,5]
    Room_y = [-3,2,7,-7,-3,1]
    rospy.loginfo('moving at the center of the arena')
    #reach the centre of the arena with move_nase
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    #wait for server
    client.wait_for_server()
    #fill the message
    msg=MoveBaseGoal()
    msg.target_pose.header.frame_id="map"
    msg.target_pose.pose.position.x=0.0
    msg.target_pose.pose.position.y=-1.0
    msg.target_pose.pose.position.z=0.0
    msg.target_pose.pose.orientation.x=0.0
    msg.target_pose.pose.orientation.y=0.0
    msg.target_pose.pose.orientation.z=0.0 
    msg.target_pose.pose.orientation.w=1.0       
   
    # Sends the goal to the action server.
    client.send_goal(msg)
    

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    #announce the hypothesis
    rospy.loginfo('Announce to Oracle: ')
    rospy.loginfo(req.who + ' with the ' + req.what + ' in the ' + req.where)
    #return to the starting location
    #get the starting location
    actual_loc = rospy.get_param('/actual_location')

    msg.target_pose.pose.position.x = Room_x[actual_loc-1]
    msg.target_pose.pose.position.y = Room_y[actual_loc-1]
    client.send_goal(msg)
    client.wait_for_result()

    

    return True


def main():
    # init node
    rospy.init_node('announce_service')
    # init service
    srv = rospy.Service('announce_service', Announcement, announce_clbk)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
