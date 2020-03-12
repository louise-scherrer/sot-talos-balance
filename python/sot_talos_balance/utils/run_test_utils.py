"""
2019, LAAS/CNRS
@author: Gabriele Buondonno
This module contains utilities for running the tests
"""
# flake8: noqa
from __future__ import print_function

from distutils.util import strtobool
from time import sleep

import rospy
from std_srvs.srv import *

from dynamic_graph_bridge_msgs.srv import *

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

runCommandClient = rospy.ServiceProxy('run_command', RunCommand)


def runVerboseCommandClient(code, verbosity=1):
    """
    Auxiliary function to manage the verbosity of runCommandClient
    verbosity = 0: simply execute runCommandClient
    verbosity = 1 (default): if runCommandClient returns something on the standard output or standard error, print it
    verbosity = 2: print the whole answer returned by runCommandClient
    """
    if verbosity > 1:
        print(code)

    out = runCommandClient(code)

    if verbosity > 1:
        print(out)
    elif verbosity == 1 and (out.standardoutput or out.standarderror):
        print("command: " + code)
        if out.standardoutput:
            print("standardoutput: " + out.standardoutput)
        if out.standarderror:
            print("standarderror: " + out.standarderror)
    return out


def evalCommandClient(code):
    """
    Auxiliary function to quickly extract return values from runCommandClient.
    This will only work when the result in a plain object data type (such as an in or float) or a string
    """
    return eval(runCommandClient(code).result)


def launch_script(code, title, description="", verbosity=1):
    input(title + ':   ' + description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    indent = '  '
    indenting = False
    for line in code:
        if indenting:
            if line == '' or line.startswith(indent):
                codeblock += '\n' + line
                continue
            else:
                answer = runVerboseCommandClient(str(codeblock), verbosity)
                rospy.logdebug(answer)
                indenting = False
        if line != '' and line[0] != '#':
            if line.endswith(':'):
                indenting = True
                codeblock = line
            else:
                answer = runVerboseCommandClient(str(line), verbosity)
                rospy.logdebug(answer)
    rospy.loginfo("...done with " + title)


def run_test(appli, verbosity=1):
    try:
        rospy.loginfo("Waiting for run_command")
        rospy.wait_for_service('/run_command')
        rospy.loginfo("...ok")

        rospy.loginfo("Waiting for start_dynamic_graph")
        rospy.wait_for_service('/start_dynamic_graph')
        rospy.loginfo("...ok")

        runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

        initCode = open(appli, "r").read().split("\n")

        rospy.loginfo("Stack of Tasks launched")

        launch_script(initCode, 'initialize SoT', verbosity=verbosity)
        input("Wait before starting the dynamic graph")
        runCommandStartDynamicGraph()
        print()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def ask_for_confirmation(text):
    c = input(text + " [y/N] ")
    try:
        return strtobool(c)
    except Exception:
        return False


def run_ft_calibration(sensor_name, force=False):
    cb = force
    if not cb:
        cb = ask_for_confirmation("Calibrate force sensors?")
    if cb:
        print("Calibrating sensors...")
        runCommandClient(sensor_name + '.calibrateFeetSensor()')
        sleep(1.0)  # TODO: get time/state from F/T sensor
        print("Sensors are calibrated!")
        print("You can now put the robot on the ground")
    else:
        print("Skipping sensor calibration")


def run_ft_wrist_calibration(sensor_name, force=False):
    cb = False
    if force:
        cb = True
    else:
        c = input("Calibrate wrist force sensors? [y/N] ")
        try:
            cb = strtobool(c)
        except Exception:
            cb = False
    if cb:
        input("Wait before running the calibration")
        print("Calibrating sensors...")
        runCommandClient(sensor_name + '.calibrateWristSensor()')
        sleep(1.0)  # TODO: get time/state from F/T sensor
        print("Sensors are calibrated!")
    else:
        print("Skipping sensor calibration")


def get_file_folder(argv, send=True):
    if len(argv) == 1:
        test_folder = None
        sot_talos_balance_folder = False
        print('No folder data')
    elif len(argv) == 2:
        test_folder = argv[1]
        sot_talos_balance_folder = False
        print('Using folder ' + test_folder)
    elif len(argv) == 3:
        if argv[1] != '-0':
            raise ValueError("Unrecognized option: " + argv[1])
        test_folder = argv[2]
        sot_talos_balance_folder = True
        print('Using folder ' + test_folder + ' from sot_talos_balance')
    else:
        raise ValueError("Bad options")

    if send:
        print("Sending folder info...")
        if test_folder is None:
            runCommandClient('test_folder = None')
        else:
            runCommandClient('test_folder = "' + test_folder + '"')
        runCommandClient('sot_talos_balance_folder = ' + str(sot_talos_balance_folder))

    return test_folder, sot_talos_balance_folder
