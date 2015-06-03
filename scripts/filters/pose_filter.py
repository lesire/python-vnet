from .basefilter import BaseFilter

import rospy
import rostopic
from geometry_msgs.msg import * 

import math

class PoseFilter(BaseFilter):
    _name = "pose"
    
    def __init__(self, range, **kwargs):
        BaseFilter.__init__(self)
        self._range = range
        self._positions = {}
        self.update(**kwargs)
    
    def _findposition(self, data):
        try:
            return data.position
        except:
            return self._findposition(data.pose)
                    
    def _update_pose(self, data, robot):
        self._positions[robot] = self._findposition(data)
        
    def update(self, **kwargs):
        for k, v in kwargs.items():
            try:
                msgType, _, _ = rostopic.get_topic_class(v)
            except rostopic.ROSTopicException as e:
                rospy.logwarn(e)
                continue
            print("%s type is %s (robot %s)" % (v, str(msgType), k))
            rospy.Subscriber(v, msgType, self._update_pose, k)
        
    def __eq__(self, other):
        return self._range == other._range

    def filtered(self, src, tgt):
        try:
            pos_src = self._positions[src]
            pos_tgt = self._positions[tgt]
        except Exception as e:
            rospy.logwarn("Cannot get position of either %s or %s: message is filtered" % (src, tgt))
            rospy.logwarn(e)
            return True
        dx = pos_src.x - pos_tgt.x
        dy = pos_src.y - pos_tgt.y
        dz = pos_src.z - pos_tgt.z
        dist = math.sqrt( dx**2 + dy**2 )
        rospy.logdebug("Distance between %s and %s is %f" % (src, tgt, dist))
        return dist > self._range
        
