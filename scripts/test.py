#!/usr/bin/env python

import rospy
import tf

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

class Region:
    def __init__(self):
        self.__x=0
        self.__y=0
        self.__radius=1
    
    def setPose(self, x,y):
        self.__x=x
        self.__y=y

    def setRad(self, rad):
        self.__radius=rad

    def set(self, x,y,rad):
        self.setPose(x,y)
        self.setRadius(rad)

    def getX(self):
        return self.__x

    def getY(self):
        return self.__y

    def getRad(self):
        return self.__radius
        
class RegionsEditor:
    def __init__(self):
        self.server=InteractiveMarkerServer("circular_regions")
        rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        self.br = tf.TransformBroadcaster()
        self.region=Region()
        self.addMarker()

    def timerCallback(self, event):
        self.br.sendTransform((self.region.getX(), self.region.getY(), 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "region",
                "map")
        # print 'Timer called at ' + str(event.current_real.secs) \
        #     +' seconds and '+str(event.current_real.nsecs/1000000)+' mili senconds'

    def spin(self):
        rospy.spin()

    def addMarker(self):
        region_marker = InteractiveMarker()
        region_marker.header.frame_id = "map"
        region_marker.name = "region"
        region_marker.description = "Region"
        region_marker.pose.position.x=self.region.getX()
        region_marker.pose.position.y=self.region.getY()

        circle_marker = Marker()
        circle_marker.type = Marker.CYLINDER
        circle_marker.scale.x = 2*self.region.getRad()
        circle_marker.scale.y = 2*self.region.getRad()
        circle_marker.scale.z = 0.01
        circle_marker.color.g = 0.5
        circle_marker.color.a = 0.5
    
        trans_control = InteractiveMarkerControl()
        trans_control.always_visible = True
        trans_control.name = "translation"
        trans_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        trans_control.orientation.w=1;
        trans_control.orientation.x=0;
        trans_control.orientation.y=1;
        trans_control.orientation.z=0;
        trans_control.markers.append( circle_marker )

        region_marker.controls.append(trans_control);

        self.server.insert(region_marker, self.processFeedback)

        radius_marker = InteractiveMarker()
        radius_marker.header.frame_id = "region"
        radius_marker.name = "radius"
        radius_marker.description = "Changing Radius"
        radius_marker.pose.position.x=self.region.getRad()

        point_marker = Marker()
        point_marker.type = Marker.SPHERE
        point_marker.scale.x = 0.05
        point_marker.scale.y = 0.05
        point_marker.scale.z = 0.05
        point_marker.color.r = 1.0
        point_marker.color.a = 1.0

        scale_control = InteractiveMarkerControl()
        scale_control.always_visible = True
        scale_control.name = "scaling"
        scale_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        scale_control.markers.append( point_marker )

        radius_marker.controls.append(scale_control);
    
        self.server.insert(radius_marker, self.processFeedback)
        self.server.applyChanges()
 
    def processFeedback(self,feedback):
        p = feedback.pose.position
        if feedback.event_type== InteractiveMarkerFeedback.POSE_UPDATE:
            if feedback.marker_name=="region":
                if feedback.control_name=="translation":
                    self.region.setPose(p.x,p.y)
        if feedback.event_type== InteractiveMarkerFeedback.POSE_UPDATE:
            if feedback.marker_name=="radius":
                if feedback.control_name=="scaling":
                    self.region.setRad(p.x)
                    self.updateMarker("region")
        self.server.applyChanges()

    def updateMarker(self, name):
        region_marker=self.server.get(name)
        region_marker.controls[0].markers[0].scale.x=self.region.getRad()*2
        region_marker.controls[0].markers[0].scale.y=self.region.getRad()*2
        self.server.insert(region_marker, self.processFeedback)
        
if __name__=="__main__":    
    rospy.init_node("test_marker")
    regionsEdit=RegionsEditor()
    regionsEdit.spin()
