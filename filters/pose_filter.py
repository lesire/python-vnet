from .basefilter import BaseFilter

import rospy
import rostopic
from geometry_msgs.msg import * 

import math

class PoseFilter(BaseFilter):
    _name = "pose"
    
    def __init__(self, range):
        BaseFilter.__init__(self)
        self._range = range
        self._positions = {}
    
    def _finditem(obj, key):
        if key in obj.keys():
            return obj[key]
        for k, v in obj.items():
            if isinstance(v,dict):
                item = _finditem(v, key)
                    
    def _update_pose(self, data, robot):
        self._positions[robot] = _finditem(data, 'position')
        
    def update(self, **kwargs):
        for k, v in kwargs.items():
            try:
                msgType = rostopic.get_topic_class(v)
            except rostopic.ROSTopicException as e:
                rospy.logwarn(e)
                continue
            rospy.Subscriber(v, msgType, self._update_pose, k)
        
    def __eq__(self, other):
        return self._range == other._range

    def filtered(self, src, tgt):
        try:
            pos_src = self._positions[src]
            pos_tgt = self._positions[tgt]
        except:
            rospy.logwarn("Cannot get position of either %s or %s: message is filtered" % (src, tgt))
            return True
        dx = pos_src.x - pos_tgt.x
        dy = pos_src.y - pos_tgt.y
        dz = pos_src.z - pos_tgt.z
        dist = math.sqrt( dx**2 + dy**2 )
        return dist > self._range
        