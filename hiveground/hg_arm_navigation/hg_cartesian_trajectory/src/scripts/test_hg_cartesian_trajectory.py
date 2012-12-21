#!/usr/bin/env python
import roslib; roslib.load_manifest('hg_cartesian_trajectory')
import hg_cartesian_trajectory
import geometry_msgs
import rospy
from hg_cartesian_trajectory.srv import HgCartesianTrajectory
from geometry_msgs.msg import Pose
import time
import sys
import pdb
import tf
import std_msgs 
import arm_navigation_msgs
import sensor_msgs
from hg_cartesian_trajectory.srv._HgCartesianTrajectory import HgCartesianTrajectoryRequest
from sensor_msgs.msg._JointState import JointState


def call_plan_simple_ik_trajectory(positions, orientations):
    rospy.wait_for_service("cartesian_planner/plan_cartesian_path")
    
       
    header = std_msgs.msg.Header()
    header.frame_id = "/base_link"
    header.stamp = rospy.get_rostime()
        
    motion_plan_request = arm_navigation_msgs.msg.MotionPlanRequest()
    motion_plan_request.group_name = "manipulator"
    
    
    #0.279335; 0; 0.567441
    #0; -0.00436349; 0; 0.99999    
    #joint_state = sensor_msgs.msg.JointState()
    #joint_state.name = ["J1", "J2", "J3", "J4", "J5"]
    #joint_state.
    
    joint_state_message = rospy.wait_for_message("/joint_states", JointState)
    print joint_state_message
    
    motion_plan_request.start_state.joint_state = joint_state_message
       
    
    poses = []
    for (position, orientation) in zip(positions, orientations):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        poses.append(pose)
        
    print "calling cartesian_planner/plan_cartesian_path for planSimpleIKTrajectory"
    try:
        service = rospy.ServiceProxy("cartesian_planner/plan_cartesian_path", HgCartesianTrajectory)
        respond = service(header, motion_plan_request, poses, HgCartesianTrajectoryRequest.SIMPLE_IK)
    except rospy.ServiceException, e:
        print "error when calling calling cartesian_planner/plan_cartesian_path: %s"%e
        return 0;
    return respond.error_code;
    
#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])    

if __name__ == "__main__":
    rospy.init_node("test_ik_simple")
    tf_listener = tf.TransformListener()
    time.sleep(5)
    
    (trans, rot) = tf_listener.lookupTransform('base_link', 'link5', rospy.Time(0))
    print "start Cartesian pose: trans", pplist(trans), "rot", pplist(rot)
        
    #0.0202451; 0; 0.779163
    #0; -0.580703; 0; 0.814115
    positions = [[0.279335, 0.183621, 0.567441],
                 [0.0202451, 0, 0.779163], 
                 [0.281936, -0.301973, 0.269322], 
                 [0.24284, 0.0106999, 0.639683],
                 [0.281732, 0, 0.292746]]
    orientations = [[-1.07747e-07, -0.00436336, 4.70156e-10, 0.99999], 
                    [0, -0.580703, 0, 0.814115],
                    [0.271839, -0.00419902, 0.00118599, 0.962333], 
                    [0.000312668, -0.152023, 0.00265087, 0.988373],
                    [0, -0.00436332, 0, 1.0]] 
    
    call_plan_simple_ik_trajectory(positions, orientations)
    
    (trans, rot) = tf_listener.lookupTransform('base_link', 'link5', rospy.Time(0))
    print "end Cartesian pose: trans", pplist(trans), "rot", pplist(rot)
    
    #0.279335; 0; 0.567441
    #0; -0.00436349; 0; 0.99999
    
    #0.280336; 0; 0.452729
    #0; -0.00436332; 0; 0.99999
    
    #0.281732; 0; 0.292746
    #0; -0.00436332; 0; 0.99999
    
 