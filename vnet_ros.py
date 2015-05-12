import rospy

from threading import RLock
from filters.filter import create_filter
import rostopic

class VNetRos:
    def __init__(self):
        self.filters_table = {}
        self.filters_lock = RLock()
        
        self._config = rospy.get_param("vnet_config")
        rospy.loginfo("Got configuration %s" % str(self._config))
        
        self._publishers = {}
        
        for t, d in self._config.items():
            self._publishers[t] = {}
            for r, p in d.items():
                try:
                    k, _, _ = rostopic.get_topic_class(p['out'])
                    rospy.Subscriber(p['out'], k, self._forward, (r, t))
                except:
                    rospy.loginfo("No input connection for robot %s on topic %s" % (r,t))
                try:
                    k, _, _ = rostopic.get_topic_class(p['in'])
                    self._publishers[t][r] = rospy.Publisher(p['in'], k, queue_size=1)
                except:
                    rospy.loginfo("No output connection for robot %s on topic %s" % (r,t))

    def _forward(self, data, robot_and_topic):
        robot, topic = robot_and_topic        
        print("Received %s from %s on topic %s" % (str(data), robot, topic))
        for r, p in self._publishers[topic].items():
            rospy.logdebug(str(self.filters_table))
            rospy.logdebug("from %s to %s" % (robot, r))
            try:
                filters = self.filters_table[robot][r]
            except:
                filters = []
            if all(map(lambda f: f(robot, r), filters)):
                p.publish(data)

    def add_filter(self, src, tgt, fname):
        self.filters_lock.acquire()
        try:
            if not src in self.filters_table.keys():
                self.filters_table[src] = {}
            if not tgt in self.filters_table[src].keys():
                self.filters_table[src][tgt] = []
            self.filters_table[src][tgt] += [create_filter(fname)]
        except Exception as e:
            print(e)
        finally:
            self.filters_lock.release()
            
if __name__ == "__main__":
    import sys
    rospy.init_node("vnet", log_level=rospy.DEBUG)
    vnet = VNetRos()
    rospy.spin()