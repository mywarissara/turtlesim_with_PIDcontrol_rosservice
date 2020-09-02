#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import time
from beginner_tutorials.msg import NewSensor
from beginner_tutorials.srv import myService, myServiceResponse
is_init_pose = False
is_init_degree = False
turnCCW = False

initPose = []
currentPose = []

linearVel = 0.75
angularVel = 0.2

distanceLength = 2.0
angleLength = np.pi/3.0
turn = False
status = ""
save_pose = []

def poseReceived(position_data): #callback
    global currentPose
    currentPose = [position_data.x, position_data.y, position_data.theta]

error0 = 0
error1 = 0
errori = 0
P =0
I= 0
D=0
de =0
t0=0
t1=0
def moveSquare():
    global errori, error1, P,I,D,de, error0, t0, t1
    # anglRadius = (angleDegree * 2 * np.pi)/360

    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseReceived)

    turtleVel = Twist()

    rospy.init_node('moveSquare4turtleSim', anonymous=False)
    rate = rospy.Rate(20) # 20hz

    flag = 0


    while not rospy.is_shutdown():
        if isChange == True:
            if flag == 0:
                save_pose = currentPose
                t0=time.time()
                goal_sq = save_pose[0] + distanceLength
                error0 = abs(goal_sq - currentPose[0])
                flag = 1
                Dist = np.sqrt(np.power(currentPose[0]-save_pose[0],2) + np.power(currentPose[1]-save_pose[1],2))
                if Dist >= distanceLength:
                    turtleVel.linear.x = 0
                if currentPose[2] < 0:
                    currentPose[2] = np.abs(currentPose[2]) + np.pi
                if save_pose[2] < 0:
                    save_pose[2] = np.abs(save_pose[2]) + np.pi
                angleDist = np.abs(currentPose[2]-save_pose[2])
                if angleDist >= angleLength:
                    flag = 0
                    turtleVel.linear.x = 0
                    turtleVel.angular.z = 0
                else:
                    turtleVel.linear.x = 0
                    turtleVel.angular.z = angularVel
            else:
                t1=time.time()
                error1 = goal_sq - currentPose[0]
                de = (error0-error1)/(t0-t1)
                errori += ((t0-t1)*(error0-error1))
                D = Kd * de
                P = Kp * error1
                I = Ki * errori
                error0 = error1
                t0 =  t1
                print(error0)
                print(error1)
                turtleVel.linear.x = P + D + I
                # turtleVel.linear.x = linearVel
            velocity_publisher.publish(turtleVel)
            # rospy.loginfo(status)
            rate.sleep()
Kp = 5.0
Ki = 0.0
Kd = 0.5
goal = 10.0
isChange = False
def moveZero():
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseReceived)

        turtleVel = Twist()

        rospy.init_node('moveSquare4turtleSim', anonymous=False)
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            turtleVel.linear.x = 0
            turtleVel.angular.z = 0
        velocity_publisher.publish(turtleVel)

    # rospy.loginfo(status)
        rate.sleep()
def moveToPoint():
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseReceived)
    turtleVel = Twist()
    rospy.init_node('moveSquare4turtleSim', anonymous=False)
    rate = rospy.Rate(20) # 20hz
    flag = 0
    errori = 0
    while not rospy.is_shutdown():
        if flag == 0:
            t0=time.time()
            error0 = goal - currentPose[0]
            flag = 1
        else:
            t1=time.time()
            error1 = goal - currentPose[0]
            de = (error0-error1)/(t0-t1)
            errori += ((t0-t1)*(error0-error1))
            D = Kd * de
            P = Kp * error1
            I = Ki * errori
            error0 = error1
            t0 =  t1
            turtleVel.linear.x = P + D + I


        # Dist = np.sqrt(np.power(currentPose[0]-save_pose[0],2) + np.power(currentPose[1]-save_pose[1],2))
            # if currentPose[0] >= 10.0:
            #     turtleVel.linear.x = 0
            #     turtleVel.angular.z = 0
            # else:
            #     turtleVel.linear.x = 1.0
            #     turtleVel.angular.z = 0

        velocity_publisher.publish(turtleVel)

        # rospy.loginfo(status)
        rate.sleep()
def customMsg():
    student_info = NewSensor()
    pub = rospy.Publisher('/student_info', NewSensor, queue_size=10)
    rospy.init_node('pub_msg', anonymous=False)
    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        student_info.age = 10
        student_info.first_name = "warissara"
        student_info.last_name = "chawalitsoonthorn"

        pub.publish(student_info)
def start_move(request):
        global isChange
        isChange = True
        return myServiceResponse()
def stop_move(request):
        global isChange
        isChange = False
        moveZero()
        return myServiceResponse()
if __name__ == '__main__':
    try:
        # rospy.wait_for_service('myService')
        rospy.init_node('moveSquare4turtleSim', anonymous=False)

        start_srv = rospy.Service('start_srv', myService, start_move)
        stop_srv = rospy.Service('stop_srv', myService, stop_move)
        moveSquare()
        # customMsg()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
