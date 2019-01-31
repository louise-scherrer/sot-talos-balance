"""
2019, LAAS/CNRS
@author: Gabriele Buondonno
This module contains utilities for running the tests
"""
from __future__ import print_function

import rospy

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

runCommandClient = rospy.ServiceProxy('run_command', RunCommand)

def evalCommandClient(code):
    return eval(runCommandClient(code).result)

def launch_script(code,title,description = ""):
    raw_input(title+':   '+description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line != '' and line[0] != '#':
            print(line)
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print(answer)
    rospy.loginfo("...done with "+title)

def run_test(appli):
    try:
        rospy.loginfo("Waiting for run_command")
        rospy.wait_for_service('/run_command')
        rospy.loginfo("...ok")

        rospy.loginfo("Waiting for start_dynamic_graph")
        rospy.wait_for_service('/start_dynamic_graph')
        rospy.loginfo("...ok")

        runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

        initCode = open( appli, "r").read().split("\n")
    
        rospy.loginfo("Stack of Tasks launched")

        launch_script(initCode,'initialize SoT')
        raw_input("Wait before starting the dynamic graph")
        runCommandStartDynamicGraph()
        print()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
