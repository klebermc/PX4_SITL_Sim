#!/usr/bin/env python3

# Common Python libraries
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Custom libraries for PX4 and tactics
from utils.px4_utilities_FB import fcuModes, Px4Info, AccelerationSetpoints, VelocitySetpoints, PositionSetpoints,customPositionController
from TSG import Controller

plot_fig=True
TL_TIME=10
class States(Enum):
    TAKEOFF = 1
    MISSION = 2
    IDLE = 3


# Main function
def main():

    ## USER SELECTABLE PARAMETERS
    # ROS loop rate
    takeoff_pos      = np.array([-1.5,-0.0,1.0])
    intermediate_pos = np.array([-1.5,-0.0,0.0])
    t_init=0
    hz = 50.0

    dz=np.array([
                [ 0.5,  1.0, 0.0],
                [-0.5, -1.0, 0.0]])
    dzs=np.array([0.5 , 0.5])
    dz_dist=1.0

    p=np.array([ 0.0,0.0,0.0])
    v=np.array([ 0.0,0.0,0.0])
    wp=np.array([2.0,0.0,0.0])

    p_hist=np.array([0.0,0.0,0.0])
    np.copyto(p_hist,takeoff_pos)

    FB_Cont=Controller()

    ## Simulation
    t = 0
    dt = 1/hz

    ## ROS and PX4 related instantiations
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)
    rate = rospy.Rate(hz)

    px4Info = Px4Info()

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, px4Info.updateStateCallback)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, px4Info.updatePoseCallback)

    # Subscribe to drone's local velocity
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, px4Info.updateVelocityCallback)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # flight mode object
    modes = fcuModes()

    # controller object
    posSet = PositionSetpoints()
    velSet = VelocitySetpoints()
    accSet = AccelerationSetpoints()

    curState = States.TAKEOFF
    setOffboardArm(posSet,modes,rate,sp_pub,px4Info)


    lastPos = []
    while not rospy.is_shutdown():

        p_data = px4Info.pose.pose.position
        v_data = px4Info.velocity.twist.linear
        p = np.array([p_data.x, p_data.y, p_data.z])
        v = np.array([v_data.x, v_data.y, v_data.z])


        if(curState == States.TAKEOFF):
            # Take off and wait
            if(t < 1):
                print("Taking off!")
                controlMode = 'pos'
                t_init=t
                posSet.updateSp(intermediate_pos)
            else:
                if(px4Info.state.armed and px4Info.state.mode == 'OFFBOARD'):
                    m=saturateSCALAR(((t-t_init)/TL_TIME),0,1)
                    intermediate_pos[2]=m*(takeoff_pos[2])
                    posSet.updateSp(intermediate_pos)
                    print("Taking off = ",intermediate_pos[2])
                    if FB_Cont.norm(p-takeoff_pos)<0.1:
                        curState = States.MISSION
                        print("Switching to mission mode")
                else:
                    raise Exception("Error: could not takeoff!")

        if(curState == States.MISSION):
            p_hist = np.vstack((p_hist,p))
            wp[2]=p[2]
            #print("Mission Mode")
            controlMode = 'vel'
            if FB_Cont.norm(p-wp)<1:
                EndGameTrigger=1
                if FB_Cont.norm(p-wp)<0.1:
                    quit()
            else:
                EndGameTrigger=0

            #controller
            a,ug,ub,pb = FB_Cont.flocking_based_controller(p, v, wp, dz, dzs, dz_dist, EndGameTrigger)
            # accSet.updateSp(np.array([0.0,0.0,1.0]),np.array([a[0],a[1],0.0]) )
            vSP=(a*12)*dt + v
            vSP[2]=0
            vSP= saturateVECT( vSP ,0.25)
            velSet.updateSp2(np.array([0.0,0.0,1.0]), vSP )
            #print("Desired acc: ", a, " Magnitude: ", np.linalg.norm(a), " Distance Goal: ", np.linalg.norm(p-wp), " Vel: ", v )
            # print("Desired acc: ", a, " Distance Goal: ", np.linalg.norm(p-wp) )
            print("Desired vel: ", vSP, " Distance Goal: ", np.linalg.norm(p-wp) )

            if(FB_Cont.done(p,wp)):
                print("Finished mission, entering Idle mode")
                lastPos = p
                curState = States.IDLE
                t_init=t
                intermediate_pos=p

        if(curState == States.IDLE):
            print("Idle mode")
            controlMode = 'pos'
            # posSet.updateSp(wp)
            m=saturateSCALAR(((t-t_init)/TL_TIME),0,1)
            intermediate_pos[2]=(1-m)*(takeoff_pos[2])
            posSet.updateSp(intermediate_pos)
            if p[2] < 0.35:
                modes.setAutoLandMode()
                break

        if (controlMode == 'pos'):
            posSet.sp.header.stamp=rospy.get_rostime()
            sp_pub.publish(posSet.sp)
        elif(controlMode == 'vel'):
            velSet.sp.header.stamp=rospy.get_rostime()
            sp_pub.publish(velSet.sp)
        elif(controlMode == 'acc'):
            accSet.sp.header.stamp=rospy.get_rostime()
            sp_pub.publish(accSet.sp)
        t = t + dt
        rate.sleep()

    return p_hist, wp, dz, dzs

def setOffboardArm(posSet,modes,rate,sp_pub,px4Info):
    #Stabilized mode first
    k=0
    while k<5:
        print("set STABILIZED")
        modes.setStabilizedMode()
        rate.sleep()
        k = k + 1


    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    # Make sure the drone is armed
    k=0
    while k<10:
        print( "publishing!")
        pos = np.array([0,0,1])
        posSet.updateSp(pos)
        sp_pub.publish(posSet.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    k=0
    while k<5:
        print("Setting to offboard mode")
        sp_pub.publish(posSet.sp)
        modes.setOffboardMode()
        rate.sleep()
        k = k + 1

    while not px4Info.state.armed:
        print("Arming")
        modes.setArm()
        rate.sleep()

def saturate(vector,minVal,maxVal):
    for i in range(0,len(vector)):
        vector[i]=min(max(vector[i],minVal),maxVal)
    return vector

def saturateSCALAR(val,minVal,maxVal):
    val=min(max(val,minVal),maxVal)
    return val

def saturateVECT(vector,maxMag):
    mag = np.linalg.norm(vector)
    if mag > maxMag:
        vector_norm = vector / mag
        vector = vector_norm*maxMag
    return vector


if __name__ == '__main__':
    try:
        p_hist, wp, dz, dzs= main()
        bounds = np.array([-3.5, 3.5, -3.5, 3.5, 0, 2])

        if plot_fig:
            fig = plt.figure()
            ax = plt.axes()

            # TARGET
            ax.scatter(wp[0],wp[1], color = 'black')

            # TRAJECTORY UAV
            ax.plot(p_hist[:,0], p_hist[:,1], '-b')

            # DZ
            counterdz=0
            for idz in dz:
                xc=idz[0]
                yc=idz[1]
                radius = dzs[counterdz]
                angle = np.linspace( 0 , 2 * np.pi , 150 )
                x = radius * np.cos( angle )
                y = radius * np.sin( angle )
                ax.scatter(idz[0],idz[1], color = 'r')
                ax.plot(x+xc, y+yc , '-r')
                counterdz=counterdz+1

            plt.grid(color='k', linestyle='-', linewidth=0.2)
            ax.set_aspect('equal', 'box')
            ax.axis(bounds[0:4])

            plt.show()

    except rospy.ROSInterruptException:
        pass
