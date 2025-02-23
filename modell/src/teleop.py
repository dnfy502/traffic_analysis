#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
import sys, select, termios, tty

moveBindings = {
    'w': (1.0, 0, 0),
    's': (-1.0, 0, 0),
    'a': (0, 1.0, 0),
    'd': (0, -1.0, 0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('box_teleop')
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    state = ModelState()
    state.model_name = "my_box"
    state.reference_frame = "world"

    try:
        while True:
            key = getKey()
            if key in moveBindings.keys():
                state.twist.linear.x = moveBindings[key][0]
                state.twist.linear.y = moveBindings[key][1]
                state.twist.angular.z = moveBindings[key][2]
            else:
                state.twist.linear.x = 0
                state.twist.linear.y = 0
                state.twist.angular.z = 0

            pub.publish(state)
            rospy.sleep(0.1)

            if key == '\x03':
                break

    except Exception as e:
        print(e)

    finally:
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.angular.z = 0
        pub.publish(state)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
