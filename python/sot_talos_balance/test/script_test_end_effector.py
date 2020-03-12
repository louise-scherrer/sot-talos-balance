import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench

rospy.wait_for_service('/gazebo/apply_body_wrench')
apply_body_wrench_proxy = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
wrench = Wrench()
wrench.force.x = 15.0
wrench.force.y = 0.
wrench.force.z = 0.
wrench.torque.x = 0.
wrench.torque.y = 0.
wrench.torque.z = 0.
duration = -1
apply_body_wrench_proxy(body_name='arm_right_7_link', wrench=wrench, duration=rospy.Duration(duration))
