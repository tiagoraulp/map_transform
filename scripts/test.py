#!/usr/bin/env python

import rospy
import tf

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *

class IdReg:
    def __init__(self):
        self._unused_ids=[]
        self._ids_count=0
    def getNewId(self):
        if len(self._unused_ids)==0:
            self._ids_count+=1
            return self._ids_count
        else:
            return self._unused_ids.pop()
    def freeId(self, id2free):
        if id2free>0 and id2free<=self._ids_count and \
                (id2free not in self._unused_ids):
            if id2free==self._ids_count:
                self._ids_count-=1
            else:
                self._unused_ids.append(id2free)
    def getUnusedIds(self):
        return self._unused_ids
    def getUsedIds(self):
        return [item for item in range(1, self._ids_count+1) \
                    if item not in self._unused_ids] 
    def getUnusedIdsCount(self):
        return len(self._unused_ids)
    def getUsedIdsCount(self):
        return self._ids_count - len(self._unused_ids)

class Region:
    def __init__(self, new_id, x=0, y=0, rad=1):
        self._x=x
        self._y=y
        self._radius=rad
        self._id=new_id
        self._name="region"+str(self._id)
    def setPose(self, x,y):
        self._x=x
        self._y=y
    def setRad(self, rad):
        self._radius=rad
    def set(self, x,y,rad):
        self.setPose(x,y)
        self.setRadius(rad)
    def getX(self):
        return self._x
    def getY(self):
        return self._y
    def getRad(self):
        return self._radius
    def getId(self):
        return self._id
    def getName(self):
        return self._name

class RegionsEditor:
    def __init__(self):
        self.server=InteractiveMarkerServer("circular_regions")
        rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        self.br = tf.TransformBroadcaster()
        self.regions=[]
        self.ids=IdReg()
        self.menu_handler=MenuHandler()
        self.menu_handler.insert("Add Region", callback=self.processFeedback)
        self.menu_handler.insert("Delete Region", callback=self.processFeedback)
        self.addRegion()

    def timerCallback(self, event):
        for region in self.regions:
            self.br.sendTransform((region.getX(), region.getY(), 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    region.getName(),
                    "map")

    def addRegion(self, x=0, y=0):
        region=Region(self.ids.getNewId(), x, y)
        self.regions.append(region)
        self.addMarker(region)

    def deleteRegion(self, name):
        self.eraseRegion(name);
        self.eraseMarker(self.IdFromName(name))
        
    def spin(self):
        rospy.spin()

    def addMarker(self, region):
        region_marker = InteractiveMarker()
        region_marker.header.frame_id = "map"
        region_marker.name = region.getName()
        region_marker.description = "Region "+str(region.getId())
        region_marker.pose.position.x=region.getX()
        region_marker.pose.position.y=region.getY()

        circle_marker = Marker()
        circle_marker.type = Marker.CYLINDER
        circle_marker.scale.x = 2*region.getRad()
        circle_marker.scale.y = 2*region.getRad()
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

        region_marker.controls.append(trans_control)
        
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name="menu"

        region_marker.controls.append(menu_control)

        self.server.insert(region_marker, self.processFeedback)

        self.menu_handler.apply( self.server, region_marker.name )

        radius_marker = InteractiveMarker()
        radius_marker.header.frame_id = region.getName()
        radius_marker.name = "radius"+str(region.getId())
        radius_marker.description = "Changing Radius "+str(region.getId())
        radius_marker.pose.position.x=region.getRad()

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
        name=feedback.marker_name
        control=feedback.control_name
        if feedback.event_type== InteractiveMarkerFeedback.POSE_UPDATE:
            if "region" in name:
                if control=="translation":
                    self.updateRegionPose(name,p.x,p.y)
            if "radius" in name:
                if control=="scaling":
                    markId=self.IdFromName(name)
                    self.updateRegionRadius(markId,p.x)
        if feedback.event_type== InteractiveMarkerFeedback.MOUSE_UP:
            self.server.applyChanges()

        if feedback.event_type== InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id==1:
                self.addRegion(p.x+0.5, p.y+0.5)
            if feedback.menu_entry_id==2:
                self.deleteRegion(name)

    def IdFromName(self, name):
        return int("0"+name[6:])

    def findRegionByName(self, name):
        return next((region for region in self.regions \
                    if region.getName()==name),None)

    def findRegionById(self, Id):
        return next((region for region in self.regions \
                    if region.getId()==Id),None)

    def updateRegionPose(self, name, x, y):
        region=self.findRegionByName(name)
        if region:
            region.setPose(x,y)
        else:
            markId=self.IdFromName(name)
            self.eraseMarker(markId)

    def updateRegionRadius(self, Id, rad):
        region=self.findRegionById(Id)
        if region:
            region.setRad(rad)
            self.updateRegionMarkerRadius(region.getName(), rad)
        else:
            self.eraseMarker(Id)

    def updateRegionMarkerRadius(self, name, rad):
        region_marker=self.server.get(name)
        if region_marker:
            region_marker.controls[0].markers[0].scale.x=rad*2
            region_marker.controls[0].markers[0].scale.y=rad*2
            self.server.insert(region_marker, self.processFeedback)
            self.server.applyChanges()
        else:
            self.eraseRegion(name)

    def eraseRegion(self, name):
        erase=[region for region in self.regions if region.getName()==name]
        for region in erase:
            self.ids.freeId(region.getId())
            self.regions.remove(region)

    def eraseMarker(self, Id):
        self.server.erase("region"+str(Id))
        self.server.erase("radius"+str(Id))
        self.server.applyChanges()

if __name__=="__main__":    
    rospy.init_node("test_marker")
    regionsEdit=RegionsEditor()
    regionsEdit.spin()
