#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from Arm_Ctrl.srv import arm_service, arm_serviceRequest

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   Q    W    E
   A    S    D
   Z    X    C

t : up (+z)
b : down (-z)

anything else : stop

emergence Stop: r
cancel Stop: t

=/- : increase/decrease max speeds by 10%
]/[ : increase/decrease only linear speed by 10%
p/o : increase/decrease only angular speed by 10%
SPACE To shoot
h/j/k to close press open
CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        'e':(1,0,0,-1),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
        'q':(1,0,0,1),
        'x':(-1,0,0,0),
        'c':(-1,0,0,1),
        'z':(-1,0,0,-1),
        'E':(1,-1,0,0),
        'W':(1,0,0,0),
        'A':(0,1,0,0),
        'D':(0,-1,0,0),
        'Q':(1,1,0,0),
        'X':(-1,0,0,0),
        'C':(-1,-1,0,0),
        'Z':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        '=':(1.1,1.1),
        '-':(.9,.9),
        ']':(1.1,1),
        '[':(.9,1),
        'p':(1,1.1),
        'o':(1,.9),
    }
breakBindings={
    'r':(1,1),
    't':(0,0),
    'R':(1,1),
    'T':(0,0),
}

armBidings={
    ' ':(),   # shut
    'h':(),   # close
    'j':(),   # press
    'k':(),   # open
    'l':(),   # 
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def send_arm_order(key):
    req = arm_serviceRequest()
    if key == ' ':   # shoot
        req.type = 'combine_shoot'
        req.value = 0
        arm_action.call(req)
    elif key == 'h': # close 
        req.type = 'arm'
        req.value = 0
        arm_action.call(req)
    elif key == 'j':  # press
        req.type = 'combine_grab'
        req.value = 8
        arm_action.call(req)
    elif key == 'k':  # open
        req.type = 'arm'
        req.value = 0x0a
        arm_action.call(req)

    


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    breakPub = rospy.Publisher('/CarBreak', Bool, queue_size=10)
    arm_action = rospy.ServiceProxy('Arm_actions', arm_service)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.6)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in breakBindings.keys():
                if breakBindings[key][0] == 1:
                    breakPub.publish(True)
                elif breakBindings[key][0] == 0:
                    breakPub.publish(False)
                continue

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in armBidings.keys():
                send_arm_order(key)


            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
            

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

