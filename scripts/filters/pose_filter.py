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

        rospy.loginfo("Creating a pose filter with %s" % kwargs)
        self.update(**kwargs)
    
    def _findposition(self, data):
        try:
            return data.position
        except:
            return self._findposition(data.pose)
                    
    def _update_pose(self, data, robot):
        self._positions[robot] = self._findposition(data)

    # Assume that in kwargs there is the position topic as "robotName":"/robot/pose"
    def update(self, **kwargs):

        #Assume default position topic if not given
        if "src" in kwargs and kwargs["src"] not in kwargs:
            kwargs[kwargs["src"]] = "/%s/pose" % kwargs["src"]

        if "tgt" in kwargs and kwargs["tgt"] not in kwargs:
            kwargs[kwargs["tgt"]] = "/%s/pose" % kwargs["tgt"]

        for k, v in kwargs.items():
            if k in ["src","tgt"]: continue
            try:
                msgType, _, _ = rostopic.get_topic_class(v)
            except rostopic.ROSTopicException as e:
                rospy.logwarn(e)
                continue
            rospy.loginfo("%s type is %s (robot %s)" % (v, str(msgType), k))
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
        
