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
    desiredT = yaw * (180.0/math.pi)
    distance = math.sqrt(math.pow((desiredX - xPosition), 2) + math.pow((desiredY - yPosition), 2))
    adjustedX = goal.pose.position.x - xPosition
    adjustedY = goal.pose.position.y - yPosition
    print goal.pose.position.x, goal.pose.position.y
    print xPosition, yPosition
    print adjustedX, adjustedY
    initialTurn = (math.atan2(adjustedY, adjustedX) * (180 / math.pi)) - theta

    print "moving from (" + str(xPosition) + ", " + str(yPosition) + ") @ " + str(theta) + " degrees"
    print "moving to (" + str(desiredX) + ", " + str(desiredY) + ") @ " + str(desiredT) + " degrees"
    print "distance: " + str(distance) + ", initial turn: " + str(initialTurn)
    print "spin!" #turn to calculated angle
    rotateDegrees(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveSmooth(0.25, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle
    finalTurn = desiredT - theta
    print "rotate " + str(finalTurn) +  " to " + str(desiredT)
    rotateDegrees(finalTurn)
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
    global pose

    initialX = pose.pose.position.x
    initialY = pose.pose.position.y

    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            sendMoveMsg(0, 0)
        else:
            sendMoveMsg(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    global pose

    initialX = pose.pose.position.x
    initialY = pose.pose.position.y
    atTarget = False
    rampSpeed = 0.0
    sleepTime = 0.05
    rampPercentage = 0.3
    step = speed / ((rampPercentage * (distance / speed)) / sleepTime)
    print "Step size: " + str(step)
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            sendMoveMsg(0, 0)
        else:
            if ((distance - currentDistance) <= distance * rampPercentage and rampSpeed >= 0):
                rampSpeed -= step
                sendMoveMsg(rampSpeed, 0)
            elif ((distance - currentDistance) >= distance * (1.0 - rampPercentage) and rampSpeed <= speed):
                rampSpeed += step
                sendMoveMsg(rampSpeed, 0)
            else:
                sendMoveMsg(speed, 0)
            rospy.sleep(sleepTime)


#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(3,-3,.1)
            else:
                spinWheels(-3,3,.1)


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
    global odom_list
    global odom_tf
    try:
        pose = msg.pose
        geo_quat = pose.pose.orientation
        q = [geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w]
        odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
                (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = yaw * (180.0/math.pi)
        xPosition = trans[0]
        yPosition = trans[1]
    except:
        print "Waiting for tf..."

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
    global odom_tf

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, navToPose, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    rospy.sleep(2)

    print "Starting Lab 2"

    # while not rospy.is_shutdown():
    #     rospy.spin()
    
    driveSmooth(0.25, 0.5)
    print "Lab 2 complete!"

