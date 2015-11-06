#!/usr/bin/env python

import rospy, tf
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist

# Add additional imports for each of the message types used


wheel_rad = 3.5 #cm
wheel_base = 23.0 #cm

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    #compute angle required to make straight-line move to desired pose
    print "spin!" #turn to calculated angle
    print "move!" #move in straight line specified distance to new pose
    print "spin!" #spin to final angle
    print "done"
    pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(1, 60)
    rotate(90)
    driveStraight(1, 45)
    rotate(-135)
    pass  # Delete this 'pass' once implemented


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    #calculate translation/rotation velocities
    #setup timer to expire when time elapsed
    #publish twist message to cmd_vel_mux
    #stop wheels when time is done
    global pub

    r = wheel_rad
    b = wheel_base

    u = (r / 2) * (u1 + u2)
    w = (r / b) * (u1 - u2)
    start = rospy.Time().secs

    move_msg = Twist()
    move_msg.linear.x = u
    move_msg.linear.z = w
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.linear.z = 0

    while(rospy.Time().secs - start < time and not rospy.is_shutdown()):
        pub.publish(move_msg)
    pub.publish(stop_msg)
    pass  # Delete this 'pass' once implemented


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    #compute time required to move distance at speed
    #setup timer to expire when time elapsed
    #spinWheels at same speed input in parameter
    #stop when timer expires
    pass  # Delete this 'pass' once implemented


#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    #compute wheel speeds and time to move to desired angle
    #spinWheels with speeds and time to get to angle
    pass  # Delete this 'pass' once implemented


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
        pass  # Delete this 'pass' once implemented


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    pose = Pose()
    #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0))
    pass # Delete this 'pass' once implemented


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node_ajlockman')

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables

    global pub
    global pose
    global odom_tf
    global odom_list

    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 2"

    #make the robot keep doing something...
    # rospy.Timer(rospy.Duration(10), timerCallback)

    # Make the robot do stuff...
    spinWheels(-0.6, -0.3, 10)

    print "Lab 2 complete!"

