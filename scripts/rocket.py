#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Accel
from std_msgs.msg import Bool
import tf
import math

class Rocket():

    def __init__(self):
        rospy.init_node('rocketNode', anonymous=False)
        rospy.on_shutdown(self.shutdowni_callback)
        markerPub = rospy.Publisher('rocket_marker_topic', Marker, queue_size=10)
        rospy.Subscriber("accel_topic", Accel, self.move_callback)
        rospy.Subscriber("rocket_hit_topic", Bool, self.hit_callback)
        tf_broadcaster = tf.TransformBroadcaster()

        self.nb_hits = 0
        self.start_time = rospy.Time.now()

        # parameters
        rocket_frame_id = rospy.get_param('~rocket_frame_id', 'rocket')
        rocket_parent_frame_id = rospy.get_param('~rocket_parent_frame_id', 'not_so_deep_space')
        self.initX = rospy.get_param("initX", 0.0)
        self.initY = rospy.get_param("initY", 0.0)


        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = rocket_frame_id

        self.robotMarker.header.stamp    = rospy.get_rostime()
        self.robotMarker.ns = "rocket_marker"
        self.robotMarker.id = 0
        self.robotMarker.type = 3 # cylinder
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0.0
        self.robotMarker.pose.position.y = 0.0
        self.robotMarker.pose.position.z = 1

        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 6.0
        self.robotMarker.scale.y = 2
        self.robotMarker.scale.z = 4.0

        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 1.0
        self.robotMarker.color.a = 1.0
        
        self.robotMarker.lifetime = rospy.Duration(0.0)
        self.robotMarker.frame_locked = True
        
        self.robotShield = Marker()
        self.robotShield.header.frame_id = rocket_frame_id

        self.robotShield.header.stamp    = rospy.get_rostime()
        self.robotShield.ns = "rocket_marker"
        self.robotShield.id = 1
        self.robotShield.type = 2 # sphere
        self.robotShield.action = 0
        self.robotShield.pose.position.x = 0.0
        self.robotShield.pose.position.y = 0.0
        self.robotShield.pose.position.z = 0.0

        self.robotShield.pose.orientation.x = 0
        self.robotShield.pose.orientation.y = 0
        self.robotShield.pose.orientation.z = 0
        self.robotShield.pose.orientation.w = 1.0
        self.robotShield.scale.x = 8.0
        self.robotShield.scale.y = 8.0
        self.robotShield.scale.z = 8.0

        self.robotShield.color.r = 235/255.
        self.robotShield.color.g = 213/255.
        self.robotShield.color.b = 52/255.
        self.robotShield.color.a = 1.0
        
        self.robotShield.lifetime = rospy.Duration(0.0)
        self.robotShield.frame_locked = True

        self.robotText = Marker()
        self.robotText.header.frame_id = rocket_frame_id

        self.robotText.header.stamp    = rospy.get_rostime()
        self.robotText.ns = "rocket_marker"
        self.robotText.id = 2
        self.robotText.type = 9 # text
        self.robotText.action = 0
        self.robotText.pose.position.x = 0.0
        self.robotText.pose.position.y = 0.0
        self.robotText.pose.position.z = 8.

        self.robotText.pose.orientation.x = 0
        self.robotText.pose.orientation.y = 0
        self.robotText.pose.orientation.z = 0
        self.robotText.pose.orientation.w = 1.0
        self.robotText.scale.z = 5

        self.robotText.color.r = 1.0
        self.robotText.color.g = 1.0
        self.robotText.color.b = 1.0
        self.robotText.color.a = 0.0
        self.robotText.text = "Ouch!"
        
        self.robotText.lifetime = rospy.Duration(0.0)
        self.robotText.frame_locked = True

        # initial starting location I might want to move to the param list
        self.x = self.initX
        self.y = self.initY
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        refresh_rate = 25.0

        rate = rospy.Rate(refresh_rate)
        dT = 1.0/refresh_rate

        while not rospy.is_shutdown():
            self.x += dT * self.vx
            self.y += dT * self.vy
            self.vx -= self.vx * 0.01    #friction
            self.vy -= self.vy * 0.01

            if abs(self.x) >= 50:
                self.vx = self.vx * -1
            if abs(self.y) >= 50:
                self.vy = self.vy * -1

            self.robotMarker.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotMarker)
            
            if self.robotShield.color.a > 0.0:
                self.robotShield.color.a -= 0.01

            self.robotShield.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotShield)

            if self.robotText.color.a > 0.0:
                self.robotText.color.a -= 0.01

            self.robotText.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotText)

            tf_broadcaster.sendTransform((self.x, self.y, 0.0),
                    tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                    rospy.Time.now(),
                    rocket_frame_id,
                    rocket_parent_frame_id)

            rate.sleep()

    def move_callback(self, msg):
        self.vx += math.cos(self.yaw)*msg.linear.x*0.2
        self.vy += math.sin(self.yaw)*msg.linear.x*0.2
        self.yaw += msg.angular.z*0.05
    
    def hit_callback(self, msg):
        self.robotText.color.a = 1.0
        self.robotShield.color.a = 1.0
        self.nb_hits += 1

    def shutdowni_callback(self):
        dt = (rospy.Time.now() - self.start_time).to_sec()
        rospy.logfatal("Your rocket was hit %.2f per sec!" % (self.nb_hits/dt))

if __name__ == '__main__':
    try:
        Rocket()
    except rospy.ROSInterruptException:
        pass

