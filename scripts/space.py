#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import tf
import numpy as np

class Not_so_deep_space():

    def __init__(self):
        rospy.init_node('deepSpaceNode', anonymous=False)
        markerPub = rospy.Publisher('space_marker_topic', Marker, queue_size=10)
        hitPub = rospy.Publisher('final_frontier_hit_topic', Bool, queue_size=10)
        listener = tf.TransformListener()
        
        # parameters
        space_frame_id = rospy.get_param('~space_frame_id', 'not_so_deep_space')
        rocket_frame_id = rospy.get_param('~rocket_frame_id', 'rocket')
        border_dist = 50

        rate = rospy.Rate(1)

        self.planeMarker = Marker()
        self.planeMarker.header.frame_id = space_frame_id
        self.planeMarker.header.stamp = rospy.get_rostime()
        self.planeMarker.ns = "space_markers"
        self.planeMarker.id = 1
        self.planeMarker.type = 1  # cube
        self.planeMarker.action = 0
        self.planeMarker.pose.position.x = 0
        self.planeMarker.pose.position.y = 0
        self.planeMarker.pose.position.z = 0
        self.planeMarker.pose.orientation.x = 0
        self.planeMarker.pose.orientation.y = 0
        self.planeMarker.pose.orientation.z = 0
        self.planeMarker.pose.orientation.w = 1.0
        self.planeMarker.scale.x = border_dist * 2.
        self.planeMarker.scale.y = border_dist * 2.
        self.planeMarker.scale.z = 0.1

        self.planeMarker.color.r = 0.0
        self.planeMarker.color.g = 1.0
        self.planeMarker.color.b = 0.0
        self.planeMarker.color.a = 1.0
        
        self.planeMarker.lifetime = rospy.Duration(0.0)
        self.planeMarker.frame_locked = True



        self.border1Marker = Marker()
        self.border1Marker.header.frame_id = space_frame_id
        self.border1Marker.header.stamp = rospy.get_rostime()
        self.border1Marker.ns = "space_markers"
        self.border1Marker.id = 2
        self.border1Marker.type = 1  # cube
        self.border1Marker.action = 0
        self.border1Marker.pose.position.x = 0
        self.border1Marker.pose.position.y = border_dist
        self.border1Marker.pose.position.z = 2
        self.border1Marker.pose.orientation.x = 0
        self.border1Marker.pose.orientation.y = 0
        self.border1Marker.pose.orientation.z = 0
        self.border1Marker.pose.orientation.w = 1.0
        self.border1Marker.scale.x = border_dist * 2.
        self.border1Marker.scale.y = 0.1
        self.border1Marker.scale.z = 4

        self.border1Marker.color.r = 1.0
        self.border1Marker.color.g = 0.0
        self.border1Marker.color.b = 0.0
        self.border1Marker.color.a = 1.0

        self.border1Marker.lifetime = rospy.Duration(0.0)
        self.border1Marker.frame_locked = True



        self.border2Marker = Marker()
        self.border2Marker.header.frame_id = space_frame_id
        self.border2Marker.header.stamp = rospy.get_rostime()
        self.border2Marker.ns = "space_markers"
        self.border2Marker.id = 3
        self.border2Marker.type = 1  # cube
        self.border2Marker.action = 0
        self.border2Marker.pose.position.x = 0
        self.border2Marker.pose.position.y = -border_dist
        self.border2Marker.pose.position.z = 2
        self.border2Marker.pose.orientation.x = 0
        self.border2Marker.pose.orientation.y = 0
        self.border2Marker.pose.orientation.z = 0
        self.border2Marker.pose.orientation.w = 1.0
        self.border2Marker.scale.x = border_dist * 2.
        self.border2Marker.scale.y = 0.1
        self.border2Marker.scale.z = 4

        self.border2Marker.color.r = 1.0
        self.border2Marker.color.g = 0.0
        self.border2Marker.color.b = 0.0
        self.border2Marker.color.a = 1.0

        self.border2Marker.lifetime = rospy.Duration(0.0)
        self.border2Marker.frame_locked = True



        self.border3Marker = Marker()
        self.border3Marker.header.frame_id = space_frame_id
        self.border3Marker.header.stamp = rospy.get_rostime()
        self.border3Marker.ns = "space_markers"
        self.border3Marker.id = 4
        self.border3Marker.type = 1  # cube
        self.border3Marker.action = 0
        self.border3Marker.pose.position.x = border_dist
        self.border3Marker.pose.position.y = 0
        self.border3Marker.pose.position.z = 2
        self.border3Marker.pose.orientation.x = 0
        self.border3Marker.pose.orientation.y = 0
        self.border3Marker.pose.orientation.z = 0
        self.border3Marker.pose.orientation.w = 1.0
        self.border3Marker.scale.x = 0.1
        self.border3Marker.scale.y = border_dist * 2.
        self.border3Marker.scale.z = 4

        self.border3Marker.color.r = 1.0
        self.border3Marker.color.g = 0.0
        self.border3Marker.color.b = 0.0
        self.border3Marker.color.a = 1.0

        self.border3Marker.lifetime = rospy.Duration(0.0)
        self.border3Marker.frame_locked = True



        self.border4Marker = Marker()
        self.border4Marker.header.frame_id = space_frame_id
        self.border4Marker.header.stamp = rospy.get_rostime()
        self.border4Marker.ns = "space_markers"
        self.border4Marker.id = 5
        self.border4Marker.type = 1  # cube
        self.border4Marker.action = 0
        self.border4Marker.pose.position.x = -border_dist
        self.border4Marker.pose.position.y = 0
        self.border4Marker.pose.position.z = 2
        self.border4Marker.pose.orientation.x = 0
        self.border4Marker.pose.orientation.y = 0
        self.border4Marker.pose.orientation.z = 0
        self.border4Marker.pose.orientation.w = 1.0
        self.border4Marker.scale.x = 0.1
        self.border4Marker.scale.y = border_dist * 2.
        self.border4Marker.scale.z = 4

        self.border4Marker.color.r = 1.0
        self.border4Marker.color.g = 0.0
        self.border4Marker.color.b = 0.0
        self.border4Marker.color.a = 1.0

        self.border4Marker.lifetime = rospy.Duration(0.0)
        self.border4Marker.frame_locked = True

        while not rospy.is_shutdown():
            try:
                (trans, _) = listener.lookupTransform(
                        space_frame_id, rocket_frame_id, rospy.Time(0))
                rocket_dist = np.abs(np.array(trans)) - border_dist
                if (rocket_dist >  -8.).any():
                    hitPub.publish(True)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.planeMarker.header.stamp    = rospy.get_rostime()
            self.border1Marker.header.stamp = rospy.get_rostime()
            self.border2Marker.header.stamp = rospy.get_rostime()
            self.border3Marker.header.stamp = rospy.get_rostime()
            self.border4Marker.header.stamp = rospy.get_rostime()

            #markerPub.publish(self.planeMarker)
            markerPub.publish(self.border1Marker)
            markerPub.publish(self.border2Marker)
            markerPub.publish(self.border3Marker)
            markerPub.publish(self.border4Marker)

            if(hitPub.get_num_connections() == 0):
                rospy.logwarn_once("message from space: nobody is listening!")
            else:
                rospy.loginfo_once("message from space: at least someone is around")

            rate.sleep()

if __name__ == '__main__':
    try:
        Not_so_deep_space()
    except rospy.ROSInterruptException:
        pass

