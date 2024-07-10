#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest


def distance_3d(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + 
                     (point2[1] - point1[1])**2 + 
                     (point2[2] - point1[2])**2)

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

current_local_pos = [0, 0, 0]

def local_pos_cb(msg): #read current local position and save in current_local_pos
    global current_local_pos
    current_local_pos[0] = msg.pose.position.x
    current_local_pos[1] = msg.pose.position.y
    current_local_pos[2] = msg.pose.position.z
    #rospy.loginfo("X : " + str(current_local_pos[0]) +
                  #"Y : " + str(current_local_pos[1]) +
                  #"Z : " + str(current_local_pos[2]))

waypoint_list = [[0, 0, 3],
                 [0, 5, 3],
                 [5, 5, 3],
                 [5, 0, 3],
                 [0, 0, 3]]

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_pos_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/land")
    landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
    
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    while(current_state.mode != "OFFBOARD"):
        rate.sleep()
    
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    arming_client(arm_cmd)
    
    desired_pose = PoseStamped()

    desired_pose.pose.position.x = 0
    desired_pose.pose.position.y = 0
    desired_pose.pose.position.z = -0.01

    for i in range(100):
        if(rospy.is_shutdown()):
            break
        
        local_pos_pub.publish(desired_pose)
        rospy.loginfo("setting init setpoint")
        rate.sleep()

    current_waypoint_order = 0
    rospy.loginfo("current waypoint order is 0")
    distance_error = distance_3d(waypoint_list[current_waypoint_order], current_local_pos)
    rospy.loginfo("current waypoint order is" + str(current_waypoint_order))
    tolerance = 0.1

    while(not rospy.is_shutdown()):
        
        while distance_error > tolerance:
            distance_error = distance_3d(waypoint_list[current_waypoint_order], current_local_pos)
            desired_pose.pose.position.x = waypoint_list[current_waypoint_order][0]
            desired_pose.pose.position.y = waypoint_list[current_waypoint_order][1]
            desired_pose.pose.position.z = waypoint_list[current_waypoint_order][2]
            local_pos_pub.publish(desired_pose)
            rospy.loginfo("setpoint_pos_publish")
            rospy.loginfo("distance error is : " + str(desired_pose.pose.position.z))
            rospy.loginfo("distance error is : " + str(distance_error))
            rate.sleep()
        
        if current_waypoint_order < len(waypoint_list)-1:
            current_waypoint_order += 1
            distance_error = 10
            rospy.loginfo("waypoint reached")
        
        else:
            land_request = CommandTOLRequest()
            land_request.min_pitch = 0
            land_request.altitude = 0
            land_request.yaw = 0
            landing_client(land_request)

    