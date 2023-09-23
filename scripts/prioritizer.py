import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sys, select, termios, tty
from math import sqrt, radians
from rospy import Time, Duration
import random  

# Initialize variables to keep track of distance traveled
distance_traveled = 0.0
foot_distance = 0.3048  # 1 foot in meters
initial_x_position = None
initial_y_position = None

def startRobot():
    global distance_traveled, initial_x_position, initial_y_position
    rospy.init_node('startRobot', anonymous=False)
    rospy.loginfo("To stop TurtleBot, press CTRL + C")
    rospy.on_shutdown(shutdown)
    
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    bumper_sub = rospy.Subscriber('mobile_base/sensors/bumper_pointcloud', PointCloud2, bumper_callback)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    turning = False
    turning_start_time = None
    turning_duration = Duration(secs=2)
    randomAngle = 0
    
    r = rospy.Rate(10)
    move_cmd = Twist()

    while not rospy.is_shutdown():
        # Check for bumper collision and halt if collision detected
        if bumper_collision:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            shutdown()
        elif turning:
            # Turn randomly between -15 and 15 degrees
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = randomAngle  # Random angle in radians
            cmd_vel.publish(move_cmd)
            if rospy.Time.now() - turning_start_time > turning_duration:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0  # Stop turning after 2 seconds
                turning = False
                cmd_vel.publish(move_cmd)
        else:
            key = getKey()

            if key == 'w':
                move_cmd.linear.x = 0.2  # Move forward
                move_cmd.angular.z = 0.0
            elif key == 's':
                move_cmd.linear.x = -0.2  # Move backward
                move_cmd.angular.z = 0.0
            elif key == 'a':
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5  # Turn left
            elif key == 'd':
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5  # Turn right
            else:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0  # Stop
        
        cmd_vel.publish(move_cmd)
        r.sleep()

        # Check if the distance traveled exceeds 1 foot and print a message
        if distance_traveled >= foot_distance:
            rospy.loginfo("Robot has moved forward 1 foot.")
            rospy.loginfo(distance_traveled)
            distance_traveled = 0.0  # Reset distance traveled
            initial_x_position = None
            initial_y_position = None
            turning = True
            randomAngle = radians(random.uniform(-15, 15))
            turning_start_time = rospy.Time.now()

def odom_callback(odom):
    global distance_traveled, initial_x_position, initial_y_position

    if initial_x_position is None:
        initial_x_position = odom.pose.pose.position.x
    if initial_y_position is None:
        initial_y_position = odom.pose.pose.position.y

    current_x_position = odom.pose.pose.position.x
    current_y_position = odom.pose.pose.position.y
    distance_traveled = sqrt((current_x_position - initial_x_position)**2 + (current_y_position - initial_y_position)**2)

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def shutdown():
    rospy.sleep(10)

def bumper_callback(data):
    global bumper_collision
    bumper_collision = True
    rospy.loginfo("Collision detected, halting robot")
    shutdown()

if __name__ == '__main__':
    bumper_collision = False
    try:
        startRobot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")



