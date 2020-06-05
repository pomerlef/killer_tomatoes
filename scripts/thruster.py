#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
import tf
import math

class Thruster():

    def __init__(self):
        rospy.init_node('thrusterNode', anonymous=False)
        markerPub = rospy.Publisher('thruster_marker_topic', Marker, queue_size=10)
        self.accelPub = rospy.Publisher('accel_topic', Accel, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.move_callback)
        tf_broadcaster = tf.TransformBroadcaster()
        
        thruster_frame_id = rospy.get_param('~thruster_frame_id', 'thruster')
        thruster_parent_frame_id = rospy.get_param('~thruster_parent_frame_id', 'rocket')

        refresh_rate = 25.0

        rate = rospy.Rate(refresh_rate)
        dT = 1.0/refresh_rate

        self.robotFireMarker = Marker()
        self.robotFireMarker.header.frame_id = thruster_frame_id
        self.robotFireMarker.header.stamp = rospy.get_rostime()
        self.robotFireMarker.ns = "game_markers"
        self.robotFireMarker.id = 6
        self.robotFireMarker.type = 3  # cylinder
        self.robotFireMarker.action = 0
        self.robotFireMarker.pose.position.x = 0.0
        self.robotFireMarker.pose.position.y = 0.0
        self.robotFireMarker.pose.position.z = 1

        self.robotFireMarker.pose.orientation.x = 0
        self.robotFireMarker.pose.orientation.y = 0.7071068
        self.robotFireMarker.pose.orientation.z = 0
        self.robotFireMarker.pose.orientation.w = 0.7071068
        self.robotFireMarker.scale.x = 1.0
        self.robotFireMarker.scale.y = 1.0
        self.robotFireMarker.scale.z = 3.0

        self.robotFireMarker.color.r = 235.0/255.0
        self.robotFireMarker.color.g = 73.0/255.0
        self.robotFireMarker.color.b = 52.0/255.0
        self.robotFireMarker.color.a = 1.0

        self.robotFireMarker.lifetime = rospy.Duration(0.0)
        self.robotFireMarker.frame_locked = True
        
        self.thrust = 0.0
        self.thrust_zeroing_counter = 0

        while not rospy.is_shutdown():

            self.thrust_zeroing_counter += 1
            if self.thrust_zeroing_counter > refresh_rate/2:
                self.thrust_zeroing_counter = refresh_rate/2
                self.thrust = 0.0

            self.robotFireMarker.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotFireMarker)


            tf_broadcaster.sendTransform((-self.thrust*2, 0.0, 1.0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    thruster_frame_id,
                    thruster_parent_frame_id)

            rate.sleep()

    def move_callback(self, msg):
        self.thrust = msg.linear.x
        self.thrust_zeroing_counter = 0
        
        acc = Accel()
        acc.linear = msg.linear
        acc.angular = msg.angular

        self.accelPub.publish(acc)


if __name__ == '__main__':
    try:
        Thruster()
    except rospy.ROSInterruptException:
        pass

