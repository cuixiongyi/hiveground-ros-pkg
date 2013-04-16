import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

import roslib; roslib.load_manifest("interactive_markers")
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import geometry_msgs.msg
import std_msgs.msg

class MarkerInfo:
    def __init__(self, name):
        self.name = name


class InteractiveProbe(Plugin):
       
    def __init__(self, context):
        super(InteractiveProbe, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InteractiveProbe')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_interactive_probe'), 'resource', 'InteractiveProbe.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('InteractiveProbeUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        
        self._widget.addMarkerPushButton.clicked.connect(self._addMarker)          
        
        
        context.add_widget(self._widget)
        
        
        self.server = InteractiveMarkerServer("InteractiveProbe")
        self.fixed_frame = "/base"              
        self.marker_id = 0
        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        rospy.loginfo("shutdown")
        self.server.clear()
        self.server.applyChanges()        

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass        

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass        

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
           
        
    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()

      
    def _addMarker(self):        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self._widget.x_spin_box.value()
        pose.position.y = self._widget.y_spin_box.value()
        pose.position.z = self._widget.z_spin_box.value()        
        scale = self._widget.scale_spin_box.value()
        name = "marker" + str(self.marker_id)
        self.addMaker(name, pose, False, scale*2, scale)
        
        """
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self._widget.x_spin_box.value()
        pose.position.y = self._widget.y_spin_box.value()
        pose.position.z = self._widget.z_spin_box.value()
        scale = self._widget.scale_spin_box.value()
        name = "marker" + str(self.marker_id)
        self.make6DofMarker( pose, scale, name, name, False)
        self.server.applyChanges()
        self.marker_id = self.marker_id + 1
        self.marker_list[name] = MarkerInfo(name)
        self.selected_marker = name
        """
        
    def addMaker(self, name, pose, selectable, length, scale):
        int_marker = InteractiveMarker()
        int_marker.name = name
        int_marker.description = name
        int_marker.header.frame_id = self.fixed_frame
        int_marker.scale = scale;
        int_marker.pose = pose;
        
        markers = []
        color = std_msgs.msg.ColorRGBA()
        color.r = 0.5
        color.g = 0.5
        color.b = 0.5
        color.a = 0.8 
        markers.append(self.makeBox(int_marker, color))                
        markers.append(self.makeArrow(int_marker, length, color))
        
        if selectable:
            self.makeSelectableControl(int_marker, markers)
        else:
            self.makeFreeMoveControl(int_marker, markers)                      
        
        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()
    

    def makeSelectableControl(self, msg, markers):
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.interaction_mode = InteractiveMarkerControl.BUTTON               
        control.markers = markers 
        msg.controls.append(control)
        return control
  
    def makeFreeMoveControl(self, msg, markers):
        control = InteractiveMarkerControl()
        control.always_visible = True
        #control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers = markers
        msg.controls.append(control)        
        return control
    
    def selectMarker(self, name):
        rospy.loginfo("Select " + name)
        
    def deselectMarker(self, name):
        rospy.loginfo("Deselect " + name)

        
    def makeBox(self, msg, color):
        marker = Marker()    
        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color = color           
        return marker
    
    def makeArrow(self, msg, length, color):
        marker = Marker()    
        marker.type = Marker.ARROW
        marker.scale.x = length
        marker.scale.y = msg.scale * 0.1
        marker.scale.z = msg.scale * 0.1
        marker.color = color           
        return marker
    
    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers.append(self.makeBox(msg))
        msg.controls.append(control)        
        return control
        
    def make6DofMarker(self, pose, scale, name, description, fixed):        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base"
        int_marker.pose = pose       
        int_marker.scale = scale
            
        int_marker.name = name
        int_marker.description = description
    
        # insert a box
        self.makeBoxControl(int_marker)
    
        if fixed:
            int_marker.name += "_fixed"
            int_marker.description += "\n(fixed orientation)"
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        self.server.insert(int_marker, self.processFeedback)
        
        
        
        
        
        
        
