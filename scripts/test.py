#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import tf
import inspect


rospy.init_node("test_marker")
server=InteractiveMarkerServer("test_interactive_marker")


def processBaseUpdate(x, y):
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "map") 
    print x
    print y
   
def processRadiusUpdate(x):
    int_mark = InteractiveMarker()
    global server
    int_mark=server.get("base")
    int_mark.controls[0].markers[0].scale.x=x
    int_mark.controls[0].markers[0].scale.y=x
    server.erase("base")
    server.insert(int_mark, processFeedback)
    server.applyChanges()
    print x

def processFeedback(feedback):
    p = feedback.pose.position
    if feedback.event_type== InteractiveMarkerFeedback.POSE_UPDATE:
        if feedback.marker_name=="base":
            if feedback.control_name=="base_control":
                processBaseUpdate(p.x,p.y)
        if feedback.marker_name=="radius":
            if feedback.control_name=="radius_control":
                print p.x
                print p.y
                processRadiusUpdate(p.y*2)

if __name__=="__main__":
    

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = "base"
    int_marker.description = "Base circle"
    int_marker.scale = 1.0

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CYLINDER
    box_marker.scale.x = 1
    box_marker.scale.y = 1
    box_marker.scale.z = 0.01
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.0
    box_marker.color.a = 0.5

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    box_control.name = "base_control"
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

    box_control.orientation.w=1;
    box_control.orientation.x=0;
    box_control.orientation.y=1;
    box_control.orientation.z=0;

    int_marker.controls.append(box_control);

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "radius"
    int_marker.description = "Changing Radius"
    int_marker.scale = 1.0
    int_marker.pose.position.y=0.5

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 1.0
    box_marker.color.g = 0.0
    box_marker.color.b = 0.0
    box_marker.color.a = 1.0

 
    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    box_control.name = "radius_control"
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    box_control.orientation.w=1;
    box_control.orientation.x=0;
    box_control.orientation.y=0;
    box_control.orientation.z=1;

    int_marker.controls.append(box_control);
    
    print inspect.getmembers(InteractiveMarkerServer, predicate=inspect.ismethod)    


    server.insert(int_marker, processFeedback)
    server.applyChanges()
        
    server.insert(int_marker, processFeedback)
    server.applyChanges()
    # 'commit' changes and send to all clients
    processBaseUpdate(0,0)

    rospy.spin()

