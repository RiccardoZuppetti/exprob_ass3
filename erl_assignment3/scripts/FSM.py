#! /usr/bin/env python

##  \package erl_assignment3
#   
#   \file FSM.py
#   \brief ROS node that simulate the robot behaviour and manage the simulation
#   \author Riccardo Zuppetti
#   \version 1.0
#   \date 22/08/2022
#   
#   \details
#   
#   Subscribes to: <BR>
#       /id_aruco
# 
#   Publishes to: <BR>
#       /geometry_msgs
# 
#   Client: <BR>
#       /oracle_hint
#   
#   Service: <BR>
#       None
#
#   Action Client:
#       /move_base
#

import sys
import rospy
import actionlib
import erl2.msg
import math
import time
from erl_assignment3.srv import ArmorInterface, ArmorInterfaceRequest, Marker, MarkerRequest
from std_srvs.srv import Empty, Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import random
import time


##
#   \brief Move base function
#   \param x : x-coordinate
#   \param y : y-coordinate
#   \return : None
#
#   This function is used to call the move_base action server
#

def move_base(x, y):

    global client_move_base
    client_move_base.wait_for_server()
    msg=MoveBaseGoal()
    msg.target_pose.header.frame_id="map"
    msg.target_pose.pose.position.x=x
    msg.target_pose.pose.position.y=y
    msg.target_pose.pose.position.z=0.0
    msg.target_pose.pose.orientation.x=0.0
    msg.target_pose.pose.orientation.y=0.0
    msg.target_pose.pose.orientation.z=0.0 
    msg.target_pose.pose.orientation.w=1.0

    client_move_base.send_goal(msg)  
    
    client_move_base.wait_for_result()
    
##
#   \brief Aruco callback
#   \param data : data retrieved at the /id_aruco topic
#   \return : None
#
#   This is the callback for the /id_aruco topic. It is used for saving the new perceived id.
#

def cbk_auco(data):

    global client_oracle_hint
    global perceived_id, to_check_id
    global new_id
    global check
    if (data.data in perceived_id) == False:
            perceived_id.append(data.data)
            to_check_id.append(data.data)
            new_id = data.data
            check = True

##
#   \brief Armor client function
#   \param new_id : new id perceived
#   \return : None
#
#   Inside this function when a new id is perceived the /oracle_hint service get the hint corresponding to that id.
#   Then the /armor_interface client is called in orde to add the hint to the ontology.
#   If the hint is correctly perceived it ask if there is a new consistent hypothesis
#   If there is it check if the actual consistent hypothesis is also correct
#

def armor_client(new_id):

    global client_oracle_hint, client_armor_interface
    global game_ended
    #look for the hint corresponding to the id
    rospy.wait_for_service('oracle_hint')
    m=MarkerRequest()
    m.markerId=new_id
    resp1 = client_oracle_hint(m)
    rospy.wait_for_service('armor_interface')
    #call /armor_interface service in perception mode
    msg=ArmorInterfaceRequest()
    msg.mode=3
    msg.ID=new_id
    msg.key=resp1.oracle_hint.key
    msg.value=resp1.oracle_hint.value
    resp2=client_armor_interface(msg)
    #call /armor_interface service in check consistent mode
    if resp2.success==True:
       rospy.wait_for_service('armor_interface')
       
       msg.mode=2
       msg.ID=new_id
       resp3=client_armor_interface(msg)
       #call /armor_interface service in check coorect mode
       if resp3.success==True:
            rospy.wait_for_service('armor_interface')
            msg.mode=1
            msg.ID=new_id
            resp4=client_armor_interface(msg)
            #if success is true means that the game is ended
            if resp4.success==True:
                rospy.loginfo("GAME ENDED")
                game_ended=True
             

##
#   \brief Rotate function
#   \param w : z-coordinate of the angular velocity
#   \return : None
#
#   This function is used to rotate the robot publishing an angualar velocity as Twist() message
#

def rotate(w):

       cmd=Twist()
       cmd.angular.z = w
       vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
       vel_publisher.publish(cmd)

##
#   \brief Change room function
#   \param : None
#   \return : None
#
#   This function is called each time the robot change the room.
#   The actual room is saved on parameter server
#   The move_base function is called
#

def change_room():   

     global Room_id, Room_x, Room_y
     global actual_room
     rospy.set_param('/actual_room', actual_room)
     move_base(Room_x[actual_room - 1], Room_y[actual_room - 1])


##
#   \brief Random motion
#   \param : None
#   \return : None
#
#   This function is called each time the robot is inside a room.
#   The robot performs 5 random movement with move_base, when the target is reached it rotate for at most ten seconds.
#   Meantime it check if it has perceived new id. If true it call armor_interface() function
#


def rnd_move():

     global check
     global start
     global Room_id, Room_x, Room_y
     global actual_room, new_id
     global perceived_id, to_check_id
     start_time = time.perf_counter()
     #random movement
     for i in range(1, 6):
             
             if start == False:
               rotate(0)
               #call move_base
               x = random.uniform(Room_x[actual_room - 1] - 1.5, Room_x[actual_room - 1] + 1.5)
               y = random.uniform(Room_y[actual_room - 1] - 1.5, Room_y[actual_room - 1] + 1.5)
               move_base(x,y)
               start_time = time.perf_counter()
             start=False
             actual_time = time.perf_counter()
             rotate(2)
             elapsed_time = actual_time - start_time
             #rotate for at most 10s
             while elapsed_time < 10:
                  #if there are new id to check it call armor_interface() function
                  if check == True:
                     
                     rotate(0)
                     for i in range(0,len(to_check_id)):
                          armor_client(to_check_id[i])
                     to_check_id=list()
                     rotate(2)
                     check=False
                  actual_time = time.perf_counter()
                  elapsed_time = actual_time - start_time
            



def main():

    # Centre: (0,-1)

    global client_move_base, client_motion_plan, client_armor_interface, client_oracle_hint
    global perceived_id, to_check_id
    global Room_id, Room_x, Room_y
    global actual_room, new_id
    global check, game_ended, start

    check = False
    # init node
    rospy.init_node('fsm')
    
    
    client_oracle_hint = rospy.ServiceProxy('oracle_hint', Marker)
    client_move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    rospy.Subscriber("/id_aruco", Int64, cbk_auco)
    

    # load the ontology
    rospy.wait_for_service('/armor_interface')
    client_armor_interface = rospy.ServiceProxy('/armor_interface', ArmorInterface)
    msg=ArmorInterfaceRequest()
    msg.mode=0
    resp = client_armor_interface(msg)

    #needed variables
    perceived_id = list()
    to_check_id = list()
    actual_room = 0
    Rooms=['Room1','Room2','Room3','Room4','Room5','Room6']
    Room_id = [1, 2,3,4,5,6]
    Room_x = [-4,-4,-4,5,5,5]
    Room_y = [-3,2,7,-7,-3,1]
    
    game_ended=False
    #spin since the game not ended
    rate = rospy.Rate(10)
    while game_ended == False:
            if actual_room == 6:
                 actual_room = 1
                 perceived_id = list()
            else:
                 actual_room = actual_room + 1
            rospy.loginfo('Going to '+ Rooms[actual_room-1])
            change_room()
            start = True
            rospy.loginfo('Randomly exploring room behavior starts')
            rnd_move()
            start=True



            rate.sleep()


if __name__ == '__main__':
    main()
