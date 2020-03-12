'''This module contains utilities for interacting with Gazebo.'''

import threading

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import Wrench
from tf.transformations import euler_from_quaternion

from dynamic_graph_bridge_msgs.msg import Vector as VectorMsg


def is_gazebo_present():
    list_of_topics = rospy.get_published_topics()
    for a_topic in list_of_topics:
        if a_topic[0].startswith('/gazebo'):
            return True
    return False


def apply_force(force, duration, body_name="talos::torso_2_link"):
    '''Gazebo service call for applying a force on a body.'''
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    apply_body_wrench_proxy = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    wrench = Wrench()
    wrench.force.x = force[0]
    wrench.force.y = force[1]
    wrench.force.z = force[2]
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    apply_body_wrench_proxy(body_name=body_name, wrench=wrench, duration=rospy.Duration(duration))


def vec2list(v):
    return [v.x, v.y, v.z]


def quat2list(v):
    return [v.x, v.y, v.z, v.w]


class GazeboLinkStatePublisher(threading.Thread):
    '''Utility class reading the state of a given link from Gazebo and publishing on a topic.'''
    def __init__(self, link_name, rate, euler='sxyz', local_frame=False, prefix='/sot'):
        super(GazeboLinkStatePublisher, self).__init__(name=link_name + "_publisher")
        self.daemon = True
        self.link_name = link_name
        if local_frame:
            self.reference_frame = link_name
        else:
            self.reference_frame = ''
        self.rate = rate
        self.prefix = prefix
        self.euler = euler
        self._stop = False
        rospy.init_node(self.name, anonymous=True)

    def stop(self):
        self._stop = True

    def stopped(self):
        return self._stop

    def run(self):
        topic_pos = self.prefix + '/' + self.link_name + '/position'
        topic_vel = self.prefix + '/' + self.link_name + '/velocity'

        pub_pos = rospy.Publisher(topic_pos, VectorMsg, queue_size=10)
        pub_vel = rospy.Publisher(topic_vel, VectorMsg, queue_size=10)

        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        rate = rospy.Rate(self.rate)
        while (not self.stopped()) and (not rospy.is_shutdown()):
            link_state_msg = get_link_state(link_name=self.link_name, reference_frame=self.reference_frame)
            link_state = link_state_msg.link_state

            cartesian = vec2list(link_state.pose.position)
            orientation = quat2list(link_state.pose.orientation)
            if self.euler:
                orientation = list(euler_from_quaternion(orientation, self.euler))
            position = cartesian + orientation

            velocity = vec2list(link_state.twist.linear) + vec2list(link_state.twist.angular)
            pub_pos.publish(position)
            pub_vel.publish(velocity)

            rate.sleep()
