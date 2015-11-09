#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

#send a movement message (Twist)
def sendMoveMsg(linearVelocity, angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT =  yaw * (180.0/math.pi) + 180
    distance = math.sqrt(math.pow((desiredX - xPosition), 2) + math.pow((desiredY - yPosition), 2))
    adjustedX = goal.pose.position.x - xPosition
    adjustedY = goal.pose.position.y - yPosition
    initialTurn = theta - 180 - math.atan2(adjustedY, adjustedX) * (180 / math.pi)

    print "moving from (" + str(xPosition) + ", " + str(yPosition) + ")"
    print "moving to (" + str(desiredX) + ", " + str(desiredY) + ")"
    print "distance: " + str(distance) + ", initial turn: " + str(initialTurn)
    print "spin!" #turn to calculated angle
    rotateDegrees(-initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveStraight(0.25, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle
    finalTurn = desiredT - theta
    print "rotate " + str(finalTurn) +  " to " + str(desiredT)
    rotate(finalTurn)
    print "done"


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(1, 0.6)
    rotateDegrees(-90)
    driveStraight(1, .45)
    rotate(135)
    pass  # Delete this 'pass' once implemented


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    r = wheel_rad
    b = wheel_base

    u = (r / 2) * (u1 + u2)
    w = (r / b) * (u1 - u2)
    start = rospy.Time().now().secs

    move_msg = Twist()
    move_msg.linear.x = u
    move_msg.angular.z = w
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()):
        pub.publish(move_msg)
    pub.publish(stop_msg)
    pass  # Delete this 'pass' once implemented


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global xPosition
    global yPosition
    global theta

    initialX = xPosition
    initialY = yPosition

    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentX = xPosition
        currentY = yPosition
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            sendMoveMsg(0, 0)
        else:
            sendMoveMsg(speed, 0)
            rospy.sleep(0.15)

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    kP = 0.01
    kI = 0.01
    kD = 0.01
    global xPosition
    global yPosition
    global theta

    desiredTheta = theta + angle * (180 / math.pi)
    if (desiredTheta >= 360):
        desiredTheta = desiredTheta - 360
    elif (desiredTheta <= 0):
        desiredTheta = 360 + desiredTheta

    e = theta - desiredTheta
    e_1 = 0
    e_2 = 0
    last = 0
    dT = 0.01
    k1 = 1 + dT * (kI / kP) + (kD / kP)
    k2 = -1 - 2 * ((kD / kP) / dT)
    k3 = (kD / dT) / dT
    r = rospy.Rate(100)
    while ((theta > desiredTheta + 2) or (theta < desiredTheta - 2) and not rospy.is_shutdown()):
        e_2 = e_1
        e_1 = e
        e = theta - desiredTheta
        u = kP * (k1 * e + k2 * e_1 + k3 * e_2)
        last = u
        # print str(e) + " " + str(e_1) + " " + str(e_2) + " " + str(dT)
        print e
        if (angle < 0):
            sendMoveMsg(0, u)
        else:
            sendMoveMsg(0, u)
        r.sleep()
    sendMoveMsg(0, 0)

def rotateDegrees(angle):
    rotate(angle * (math.pi / 180))

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    #assuming radius is turning radius, speed is drive speed, angle is desired final angle
    #calculate wheel speeds and time to move from current pose to final pose
    #spinWheels with time and speeds to move to correct pose
    pass  # Delete this 'pass' once implemented


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        #Stop forward motion if bumper is pressed
        print "Bumper pressed!"
        executeTrajectory()
        pass  # Delete this 'pass' once implemented


#Odometry Callback function.
def readOdom(msg):
    global pose
    global xPosition
    global yPosition
    global theta

    pose = msg.pose
    geo_quat = pose.pose.orientation
    xPosition = pose.pose.position.x
    yPosition = pose.pose.position.y
    q = [geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    theta = yaw * (180.0/math.pi) + 180


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    # pose = Pose()
    #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    # (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0))
    pass # Delete this 'pass' once implemented


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node_ajlockman')

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables

    global pub
    global pose
    global odom_list

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, navToPose, queue_size=1)
    sub = rospy.Subscriber('odom', Odometry, readOdom)
    odom_list = tf.TransformListener()

    # Use this command to make the program wait for some seconds
    rospy.sleep(2)

    print "Starting Lab 2"

    #make the robot keep doing something...
    # rospy.Timer(rospy.Duration(10), timerCallback)

    # driveStraight(0.25, 0.25)
    while (not rospy.is_shutdown()):
        rotateDegrees(90)
        rospy.sleep(5)
        rotateDegrees(-90)
        rospy.sleep(5)

    print "Lab 2 complete!"

