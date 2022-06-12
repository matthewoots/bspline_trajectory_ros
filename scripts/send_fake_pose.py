#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import PoseStamped

# launch this with
# python send_fake_pose.py idx x y z

def publisher():
    index = int(sys.argv[1])
    pub = rospy.Publisher('/drone' + str(index) + '/mavros/local_position/pose', PoseStamped, queue_size=20)

    rospy.init_node('pose_publisher' + str(index), anonymous=True)

    epoch = rospy.Time.now()
    while pub.get_num_connections() < 1:
        # Do nothing
        if (rospy.Time.now() - epoch).to_sec() > 5.0:
            print("No connections in 5s")
            return

    pose = PoseStamped()

    pose.header.stamp = rospy.Time.now()

    # in NWU frame
    pose.pose.position.x = float(sys.argv[2])
    pose.pose.position.y = float(sys.argv[3])
    pose.pose.position.z = float(sys.argv[4])
    
    pub.publish(pose)


if __name__ == '__main__':
    publisher()