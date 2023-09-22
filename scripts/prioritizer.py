import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2  # Import the PointCloud2 message type

import sys, select, termios, tty


def startRobot():
    rospy.init_node('startRobot', anonymous=False)
    rospy.loginfo("To stop TurtleBot CTRL + C")
    rospy.on_shutdown(shutdown)
    
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    bumper_sub = rospy.Subscriber('mobile_base/sensors/bumper_pointcloud', PointCloud2, bumper_callback)  # Subscribe to bumper PointCloud2 data
    
    r = rospy.Rate(10)
    move_cmd = Twist()
    
    while not rospy.is_shutdown():
        # Check for bumper collision and halt if collision detected
        if bumper_collision:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            shutdown()
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
        
        cmd_vel.publish(move_cmd)
        r.sleep()

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

# Bumper callback function to handle bumper events
def bumper_callback(data):
    global bumper_collision
    bumper_collision = True
    rospy.loginfo("Collision detected, halting robot")
    shutdown()

if __name__ == '__main__':
    bumper_collision = False  # Initialize bumper collision state


    try:
        startRobot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")








