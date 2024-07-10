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

waypoint_list = [[0, 0, 20],
                 [0, 20, 20],
                 [20, 20, 20],
                 [20, 0, 20],
                 [0, 0, 20]]

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


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    desired_pose = PoseStamped()

    desired_pose.pose.position.x = 0
    desired_pose.pose.position.y = 0
    desired_pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(desired_pose)
        rospy.loginfo("setting init setpoint")
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    vehicle_armed = False
    vehicle_offboarded = False

    while(not rospy.is_shutdown() and not (vehicle_armed and vehicle_offboarded)):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                vehicle_offboarded = True

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    vehicle_armed = True

                last_req = rospy.Time.now()
        
        local_pos_pub.publish(desired_pose)
        rate.sleep()
    rospy.loginfo("vehicle is ready")
    
    current_waypoint_order = 0
    rospy.loginfo("current waypoint order is 0")
    distance_error = distance_3d(waypoint_list[current_waypoint_order], current_local_pos)
    rospy.loginfo("current waypoint order is" + str(current_waypoint_order))
    tolerance = 1 

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
            land_request.min_pitch = 45
            land_request.altitude = 0
            land_request.yaw = 0
            landing_client(land_request)

        


            
        