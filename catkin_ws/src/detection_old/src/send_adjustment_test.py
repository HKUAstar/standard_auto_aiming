#!/usr/bin/env python3
import rospy as ros
from geometry_msgs.msg import Vector3

def talker():
    pub = ros.Publisher('auto_aiming_adjustment', Vector3, queue_size=10)
    ros.init_node('auto_aiming_adjustment_test', anonymous=True)
    while not ros.is_shutdown():
        # set adjustment W: Up, A: Left, S: Down, D: Right
        value = input('Enter adjustment value: ')
        direction = value[-1]
        if len(value) > 1:
            amount = float(value[:-1]) # amount in cm
        else:
            amount = 1
        adjustment = Vector3(0, 0, 0)
        if direction == 'w':
            adjustment = Vector3(0, 0.01 * amount, 0)
        elif direction == 'a':
            adjustment = Vector3(0.01 * amount, 0, 0)
        elif direction == 's':
            adjustment = Vector3(0, -0.01 * amount, 0)
        elif direction == 'd':
            adjustment = Vector3(-0.01 * amount, 0, 0)
        else:
            print('Exit adjustment')
            break
        
        ros.loginfo('Adjustment: ' + str(adjustment))
        pub.publish(adjustment)

if __name__ == '__main__':
    try:
        talker()
    except ros.ROSInterruptException:
        pass
