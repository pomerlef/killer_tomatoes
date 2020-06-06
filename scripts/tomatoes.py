#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import tf
import math
import random
import numpy as np

class Tomatoes():

    def create_tomato(self, i):
        marker = Marker()
        marker.header.frame_id = self.space_frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 3.
        marker.scale.y = 3.
        marker.scale.z = 3.
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = (random.random() - 0.5)*100.
        marker.pose.position.y = -50.
        marker.pose.position.z = 1
        marker.id = i

        return marker

    def __init__(self):
        rospy.init_node('tomatoesNode', anonymous=False)
        markersPub = rospy.Publisher('tomatoes_marker_topic', MarkerArray, queue_size=10)
        hitPub = rospy.Publisher('tomatoes_hit_topic', Bool, queue_size=10)
        listener = tf.TransformListener()

        # parameters
        self.space_frame_id = rospy.get_param('~space_frame_id', 'not_so_deep_space')
        self.rocket_frame_id = rospy.get_param('~rocket_frame_id', 'rocket')
        self.max_tomatoes = rospy.get_param('~max_tomatoes', 10)
        hit_radius = 4.

        markerArray = MarkerArray() 
        markerArray.markers.append(self.create_tomato(0))
        
        refresh_rate = 25.0
        tomato_speed = 0.4
        rate = rospy.Rate(refresh_rate)

        rocket_trans = np.zeros(3)
        tomato_hit_flag = np.full(self.max_tomatoes, False)

        while not rospy.is_shutdown():
            nb_tomatos = len(markerArray.markers)
            

            try:
                (trans, _) = listener.lookupTransform(
                        self.space_frame_id, self.rocket_frame_id, rospy.Time(0))
                rocket_trans = np.array(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            if random.random() > 0.95 and  nb_tomatos < self.max_tomatoes:
                markerArray.markers.append(self.create_tomato(nb_tomatos))

            for i, m in enumerate(markerArray.markers):
                m.pose.position.y += tomato_speed
                error = rocket_trans[0] - m.pose.position.x 
                m.pose.position.x += (random.random() - 0.5) + error/100.

                if m.pose.position.y > 50:
                    m.pose.position.y = -50
                    m.pose.position.x = (random.random() - 0.5)*100.

                tomato_trans = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
                dist = np.linalg.norm(rocket_trans - tomato_trans)
                if dist < hit_radius:
                    if(tomato_hit_flag[m.id] == False):
                        tomato_hit_flag[m.id] = True
                        hitPub.publish(True)
                else:
                    tomato_hit_flag[m.id] = False
            
            if(hitPub.get_num_connections() == 0):
                rospy.logwarn_once("message from killer tomatoes: nobody is listening to our hits")
            else:
                rospy.loginfo_once("message from killer tomatoes: someone is here ARG!!!")


            
            markersPub.publish(markerArray)

            rate.sleep()



if __name__ == '__main__':
    try:
        Tomatoes()
    except rospy.ROSInterruptException:
        pass

