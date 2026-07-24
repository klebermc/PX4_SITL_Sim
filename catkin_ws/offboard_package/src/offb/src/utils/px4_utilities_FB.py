#!/usr/bin/env python
# ROS python API
#
# Shared MAVROS helpers: `fcuModes` wraps the mavros service calls for
# arming/disarming and switching PX4 flight modes; `Px4Info` holds the
# latest state/pose/velocity from MAVROS topic callbacks; the
# `*Setpoints` classes build MAVLink SET_POSITION_TARGET_LOCAL_NED
# messages, each configuring `type_mask` to enable a different subset of
# fields (position-only, velocity-only, or position+acceleration).
import rospy
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import Point, PoseStamped, TwistStamped

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self, alt):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Takeoff could not be set.".format(e))

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Arm could not be set.".format(e))

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Disarm could not be set.".format(e))

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Stabilizer Mode could not be set.".format(e))

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("Trying offboard")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print("Finished trying offboard")
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Offboard Mode could not be set.".format(e))

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Altitude Mode could not be set.".format(e))

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Position Mode could not be set.".format(e))

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: {}. Autoland Mode could not be set.".format(e))

## Drone State callback
class Px4Info():
    def __init__(self):
        self.state = State()
        self.pose = PoseStamped()
        self.velocity= TwistStamped()
    def updateStateCallback(self, msg):
        self.state = msg
    def updatePoseCallback(self, msg):
        self.pose = msg
    def updateVelocityCallback(self, msg):
        self.velocity = msg

class PositionSetpoints:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        #self.sp.position.z = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    ## Update setpoint message
    def updateSp(self, pos):
        self.sp.position.x = pos[0]
        self.sp.position.y = pos[1]
        self.sp.position.z = pos[2]

class VelocitySetpoints:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111100011', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        # initial values for setpoints
        self.max_speed = 1

    ## Update setpoint message
    def updateSp(self, velocity):
        self.sp.velocity.x = velocity[0]
        self.sp.velocity.y = velocity[1]
        self.sp.velocity.z = velocity[2]

    ## Update setpoint message
    def updateSp2(self, p, vel):
        self.sp.position.x = p[0]
        self.sp.position.y = p[1]
        self.sp.position.z = p[2]
        self.sp.velocity.x = vel[0]
        self.sp.velocity.y = vel[1]
        self.sp.velocity.z = vel[2]

class AccelerationSetpoints:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010100111011', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    ## Update setpoint message
    def updateSp(self, p, acc):
        self.sp.position.x = p[0]
        self.sp.position.y = p[1]
        self.sp.position.z = p[2]
        self.sp.acceleration_or_force.x = acc[0]
        self.sp.acceleration_or_force.y = acc[1]
        self.sp.acceleration_or_force.z = 0

class customPositionController:
    def __init__(self,K, maxSpeed):
        self.K = K
        self.maxSpeed = maxSpeed

    def control(self, curPos, desPos):
        desVel = self.K*(desPos-curPos)
        desVelNorm = np.linalg.norm(desVel)
        if(desVelNorm > self.maxSpeed):
            desVel = desVel/desVelNorm*self.maxSpeed
        return desVel
