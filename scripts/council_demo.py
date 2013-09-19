#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_delivery')
import rospy
import actionlib
import geometry_msgs.msg
import tf.transformations

from pr2_common_action_msgs.msg import TuckArmsGoal

from pr2_controllers_msgs.msg import *

import yaml
import os

import pr2_delivery

import time

def pose_from_yaml(filename):
    stream = open( filename )
    pose_dict = yaml.load( stream )
    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/map'
    
    pose.pose.position.x = pose_dict['pose']['position']['x']
    pose.pose.position.y = pose_dict['pose']['position']['y']
    pose.pose.position.z = pose_dict['pose']['position']['z']
    
    pose.pose.orientation.x = pose_dict['pose']['orientation']['x']
    pose.pose.orientation.y = pose_dict['pose']['orientation']['y']
    pose.pose.orientation.z = pose_dict['pose']['orientation']['z']
    pose.pose.orientation.w = pose_dict['pose']['orientation']['w']
    
    return pose

if __name__ == '__main__':
    rospy.loginfo("starting...")
    rospy.init_node('council_demo')

    data_dir = rospy.get_param("~data_dir")
    rospy.loginfo("Using %s as data directory"%(data_dir))

    get_object_pose = pose_from_yaml(os.path.join(data_dir,'get_object_pose.yaml'))
    intro_pose = pose_from_yaml(os.path.join(data_dir,'intro_pose.yaml'))
    give_object_pose = pose_from_yaml(os.path.join(data_dir,'give_object_pose.yaml'))
    outro_pose = pose_from_yaml(os.path.join(data_dir,'outro_pose.yaml'))
    return_home_pose = pose_from_yaml(os.path.join(data_dir,'return_home_pose.yaml'))

    intro_phrase = rospy.get_param('~intro_phrase', 'Hello.')
    outro_phrase = rospy.get_param('~outro_phrase', 'Goodbye.')
    
    server = pr2_delivery.DeliverServer()

    point_head_client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
    point_head_client.wait_for_server()
    
    g = PointHeadGoal()
    g.target.point.x = 1.0
    g.target.point.y = 0.0
    g.target.point.z = 0.0
    g.min_duration = rospy.Duration(1.0)
    
    server.tuck_arms()
    
    time.sleep(3)
    
    server.navigate_to(get_object_pose)
    server.say(server.request_item_phrase)
    server.get_object()
    server.say(server.item_received_phrase)
    server.navigate_to(intro_pose)
    server.say(intro_phrase)
    server.navigate_to(give_object_pose)
    server.say(server.give_item_phrase)
    server.give_object()
    server.say(server.item_delivered_phrase)
    server.tuck_arms()
    server.navigate_to(outro_pose)

    # untuck arms
    goal = TuckArmsGoal()
    goal.tuck_left = false
    goal.tuck_right = false
    server.tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    # head up
    g.target.header.frame_id = 'head_pan_link'
    point_head_client.send_goal(g)

    server.say(outro_phrase)

    # head down
    g.target.header.frame_id = 'base_link'
    point_head_client.send_goal(g)

    server.tuck_arms()
    server.navigate_to(return_home_pose)
    
