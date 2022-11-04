#! /usr/bin/env python

##  \package erl_assignment3
#   
#   \file ArmorInterface.py
#   \brief ROS node used to interact with armor
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
#       /oracle_solution
#       /armor_interface_srv
#       /announce_service
#   
#   Service: <BR>
#       /armor_interface
#
#   Action Client:
#       None
#

import sys
import rospy
import actionlib
import erl2.msg
from erl2.msg import ErlOracle
import math
import time
from erl2.srv import Oracle
from erl_assignment3.srv import  Announcement, ArmorInterface, ArmorInterfaceResponse, AnnouncementRequest
from armor_msgs.srv import ArmorDirective
from erl_assignment3.srv import Marker, MarkerRequest, MarkerResponse
from std_msgs.msg import Int64

# mode 0 initialization
# mode 1 perception
# mode 2 consistency

##
#   \brief Ontology interaction function
#   \param command
#   \param primary_command_spec
#   \param secondary_command_spec
#   \param arg
#   \return msg : ArmorDirectiveRes
#
#   This function is used to interface with the ontology through the ARMOR action server. It get as argument all fields of the ArmorDirective message to fill.
#

def ontology_interaction(
        command,
        primary_command_spec,
        secondary_command_spec,
        arg):

    global client_armor
    rospy.wait_for_service('armor_interface_srv')

    msg = ArmorDirective()
    msg.client_name = 'tutorial'
    msg.reference_name = 'ontoTest'
    msg.command = command
    msg.primary_command_spec = primary_command_spec
    msg.secondary_command_spec = secondary_command_spec
    msg.args = arg
    
    resp = client_armor(msg)
    
    return resp

##
#   \brief Response function
#   \param st : stringe to be managed
#   \return st : arranged string
#
#   This function is used to manage the strings retrieved by ARMOR service
#

def menage_response(st):

    st = st.replace("<http://www.emarolab.it/cluedo-ontology#", "")
    st = st.replace(">", "")

    return st

##
#   \brief Load ontology function
#   \param : None
#   \return : None
#
#   This function is used to load the ontology and initialize the classes
#

def load_initialize_ontology():

    # load ontology
    ontology = rospy.get_param('ontology')
    ontology_path = rospy.get_param('ontology_path')

    r1 = ontology_interaction(
        'LOAD', 'FILE', '', [
            ontology_path, ontology, 'true', 'PELLET', 'true'])
    # add class incorrect

    r2 = ontology_interaction(
        'ADD', 'CLASS', 'CLASS', [
            'INCORRECT', 'HIPOTESIS'])
    # set all classes as mutually disjoint
    r3 = ontology_interaction('DISJOINT', 'CLASS', '', ['PERSON', 'PLACE'])
    r4 = ontology_interaction('DISJOINT', 'CLASS', '', ['PLACE', 'WEAPON'])
    r5 = ontology_interaction('DISJOINT', 'CLASS', '', ['WEAPON', 'PERSON'])
    r6 = ontology_interaction('DISJOINT', 'CLASS', '', ['INCORRECT', 'PERSON'])
    r7 = ontology_interaction('DISJOINT', 'CLASS', '', ['WEAPON', 'INCORRECT'])
    r8 = ontology_interaction('DISJOINT', 'CLASS', '', ['INCORRECT', 'PLACE'])

##
#   \brief Clbk function
#   \param msg : ArmorIntrfaceRequest
#   \return msg : ArmorInterfaceResponse
#
#   The node can interface in four different way with armor defendig on the mode fiel of the request.
#   If mode is 0 the ontology is loaded and initialized. 
#   If mode is 1 the current hypothesis'id is compared to the correct one. In case they are equal the game ends.
#   Mode 2 involves to check if there are new consistent hypothesis.
#   Finally mode 3 means that robot try to perceive a new hint. If perceived the hint will be loaded on the ontology.
#

def clbk(req):

    global client_oracle_solution, client_armor, client_announce
    
    #global checked
    _res = ArmorInterfaceResponse()

    # initialisation of the ontology
    if req.mode == 0:
        rospy.loginfo('Initialization')
        load_initialize_ontology()
        _res.mode = 0
        _res.success = True
        return _res
    #check if the current hypo is correct
    if req.mode == 1:
        #get the current consistent hypothesis to check
        check_id=rospy.get_param('/curr_ID')
        rospy.loginfo('CHECK CORRECT ID: '+ check_id)
        # get correct solution
        rospy.wait_for_service('oracle_solution')
        resp = client_oracle_solution()
        
        _res.mode = 1
        # announce the hypotesis
        rospy.wait_for_service('announce_service')
        current_hypotesis = rospy.get_param('current_hypotesis')
        msg = AnnouncementRequest()
        msg.who = current_hypotesis[1]
        msg.where = current_hypotesis[2]
        msg.what = current_hypotesis[3]
        
        a = client_announce(msg)
        
        #if the currecnt hypothesis is correct save the current inferenced ontology and state that the game ended
        if str(resp.ID) == check_id:
            rospy.loginfo('CORRECT')
            
            resp=ontology_interaction('SAVE','INFERENCE','',['/root/ros_ws/src/experimental_assignment3/erl_assignment3/cluedo_ontology.owl'])
            _res.success = True
            _res.ID = int(check_id)
        #if it is not correct add the incorrect hypothesis to the incorrect class 
        else:
            rospy.loginfo('NOT CORRECT')
            _res.success = False
            _res.ID = int(check_id)
            r1 = ontology_interaction('REMOVE', 'IND', '', [check_id])
            r3 = ontology_interaction(
                'ADD', 'IND', 'CLASS', [
                    check_id, 'INCORRECT'])
            r2 = ontology_interaction('REASON', '', '', [])
        return _res
    #check if there is a new consistent hypothesis
    if req.mode == 2:
        rospy.loginfo('CHECK CONSISTENCY')
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['PERSON'])
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['PLACE'])
        resp2 = ontology_interaction('DISJOINT', 'IND', 'CLASS', ['WEAPON'])
        # ask for complete hypotesis
        resp_c = ontology_interaction('QUERY', 'IND', 'CLASS', ['COMPLETED'])

        # ask for incostintent hypotesis
        resp_i = ontology_interaction(
            'QUERY', 'IND', 'CLASS', ['INCONSISTENT'])

        _res.mode = 2

        # if the length is equal means that there is not consistent
        # hypotesis to check
        if len(
                resp_i.armor_response.queried_objects) == len(
                resp_c.armor_response.queried_objects):

            _res.success = False

            rospy.loginfo('no new consistent hypothesis')
        else:
           #there is a new consistent hypothesis
            _res.success = True
            
            complete = []

            for i in range(len(resp_c.armor_response.queried_objects)):

                st = menage_response(resp_c.armor_response.queried_objects[i])
                complete.append(st)

            if len(resp_i.armor_response.queried_objects) > 0:
                for i in range(len(resp_i.armor_response.queried_objects)):
                    st = menage_response(
                        resp_i.armor_response.queried_objects[i])
                    complete.remove(st)
            # query the consistent hypothesis
            
            _res.ID = int(complete[0])
            consistent_who = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['who', complete[0]])

            who = menage_response(
                consistent_who.armor_response.queried_objects[0])
            consistent_where = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['where', complete[0]])
            where = menage_response(
                consistent_where.armor_response.queried_objects[0])
            consistent_what = ontology_interaction(
                'QUERY', 'OBJECTPROP', 'IND', ['what', complete[0]])
            what = menage_response(
                consistent_what.armor_response.queried_objects[0])

            # store the consistent hypotesis in the parameter server
            rospy.set_param('curr_ID',complete[0])
            rospy.set_param(
                'current_hypotesis', [
                    complete[0], who, where, what])
            rospy.loginfo('New consistent Hypothesis with ID'+complete[0])    
        return _res
    else:
        #perceive hint mode
        rospy.loginfo('PERCEIVE HINT')
        _res.mode = 3
        erloracle=ErlOracle()
        erloracle.key=req.key
        erloracle.ID=req.ID
        erloracle.value=req.value
        #if a field is empty or -1 the perceived hint is malformed
        if erloracle.key == '' or erloracle.value == '-1':
            rospy.loginfo('malformed hint perceived: lens is dirty!!')
            _res.success = False

        else:
            # ask the incorrect hypotesis collected
            
            r = ontology_interaction('QUERY', 'IND', 'CLASS', ['INCORRECT'])
            
            incorrect = []
            for i in range(len(r.armor_response.queried_objects)):
                st = menage_response(r.armor_response.queried_objects[i])
                incorrect.append(st)

            # if the perceived object belong to an hypotesis already checked as
            # incorrect robot will discard it
            if erloracle.ID in incorrect:
                rospy.loginfo(
                    'The perceived hint has an id associated to an incorrect hypotesis: I will discard it')
                _res.success = False

            # otherwise it add the object to the ontology
            else:
               
                #add the hint to the ontology
                _res.success = True
                r1 = ontology_interaction(
                    'ADD', 'OBJECTPROP', 'IND', [
                        erloracle.key, str(erloracle.ID), erloracle.value])
                
                if erloracle.key == 'where':
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                           erloracle.value, 'PLACE'])
                elif erloracle.key == 'what':
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                            erloracle.value, 'WEAPON'])
                else:
                    resp = ontology_interaction(
                        'ADD', 'IND', 'CLASS', [
                            erloracle.value, 'PERSON'])
                # disjoint all element of one classes is necessarily for having
                # multiple hints of a class belonging to the same hypotesis
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['PERSON'])
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['PLACE'])
                resp2 = ontology_interaction(
                    'DISJOINT', 'IND', 'CLASS', ['WEAPON'])
                # reason about the class
                r2 = ontology_interaction('REASON', '', '', [])
                rospy.loginfo('PERCEIVED: ['+str(erloracle.ID) + ':' + erloracle.value + ',' + erloracle.key + ']')
                checked=True
        return _res
        


def main():
    global client_armor,client_oracle_solution,client_announce, client_oracle_hint
    
    # init node
    rospy.init_node('armor_interface_client')
    # init service
    srv = rospy.Service('/armor_interface', ArmorInterface, clbk)
    client_oracle_solution = rospy.ServiceProxy('oracle_solution', Oracle)
    client_armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    client_announce = rospy.ServiceProxy('announce_service', Announcement)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
