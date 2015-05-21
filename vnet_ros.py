import rospy
import rostopic
from std_msgs.msg import String

from vnet import VNet
import json

def all_pairs(l):
    L = list(l)
    while L:
        i = L.pop()
        for j in L: yield i, j

'''
ROS-based implementation of VNet

The VNet ROS node takes as a configuration the 'vnet_config' parameters 
that must contain a list of 'topics' to manage. For each topic, it describes
for each involved agent, the actual input and output topic to connect to.
Example:

plan_repair: {
    mana: {in: /mana/repair/in, out: /mana/repair/out},
    ressac: {in: /ressac/repair/in, out: /ressac/repair/out}
  },
plan_update: {
  ...
}

In this example, VNet will manage two topics (plan_repair and plan_update). For plan_repair, 
VNet will subscribe to /mana/repair/out and /ressac/repair/out.
Received messages will be forwarded to all input topics of the plan_repair topic, i.e.
/mana/repair/in and /ressac/repair/in, depending on the actual filtering setup.

The VNet node provides a simple administration interface through specific topics.
The admin topics are all waiting a String object formatted as a json dict.

- /vnet/add: to add a new filter to VNet; 
  arguments: 
    - 'src': the source agent
    - 'tgt': the target agent
    - 'filter': the filter name
    - additional filter named args

- /vnet/del: to delete a filter from VNet; 
  arguments: 
    - 'src': the source agent
    - 'tgt': the target agent
    - 'index': the filter index on the filter list
    - 'filter': the filter name

- /vnet/list: to list filters for a pair of agent; 
  arguments: 
    - 'src': the source agent
    - 'tgt': the target agent

- /vnet/listall: to list all VNet filters

'''
class VNetRos(VNet):
    def __init__(self):
        VNet.__init__(self)
        
        self._config = rospy.get_param("vnet/config")
        rospy.loginfo("Got configuration %s" % str(self._config))
                
        # Administration services
        rospy.Subscriber("/vnet/add", String, self._admin, self.add_filter)
        rospy.Subscriber("/vnet/del", String, self._admin, self.del_filter)
        rospy.Subscriber("/vnet/list", String, self._admin, self.list_filters)
        rospy.Subscriber("/vnet/listall", String, self._admin, self.list_all_filters)
        
        # Statistics
        self._stat_publisher = rospy.Publisher("/vnet/statistics", String, queue_size=1)
        
        self._publishers = {}
        self._graph_publishers = {}
        self.graphs = {}

        for t, d in self._config.items():
            self._publishers[t] = {}
            for r, p in d.items():
                try:
                    k, _, _ = rostopic.get_topic_class(p['out'])
                    rospy.Subscriber(p['out'], k, self._forward, (r, t))
                except Exception as e:
                    rospy.loginfo("No input connection for robot %s on topic %s" % (r,t))
                    rospy.loginfo(e)
                try:
                    k, _, _ = rostopic.get_topic_class(p['in'])
                    self._publishers[t][r] = rospy.Publisher(p['in'], k, queue_size=1)
                except Exception as e:
                    rospy.loginfo("No output connection for robot %s on topic %s" % (r,t))
                    rospy.loginfo(e)
            self._graph_publishers[t] = rospy.Publisher("/vnet/graph/"+t, String, queue_size=1)
            self._init_graph(t, list(d.keys()))

    def _admin(self, args, request):
        try:
            if len(args.data) < 1:
                r = request()
            else:
                import json
                r = request(**json.loads(args.data))
            rospy.loginfo(r)
        except Exception as e:
            rospy.logerr(e)

    def _forward(self, data, robot_and_topic):
        robot, topic = robot_and_topic
        rospy.logdebug("Received %s from %s on topic %s" % (str(data), robot, topic))
        stats = {'from': robot, 'topic': topic, 'filtered': [], 'forwarded': [], 'data': str(data)[:20], 'size': len(str(data))}
        for r, p in self._publishers[topic].items():
            rospy.logdebug(str(self.filters_table))
            rospy.logdebug("from %s to %s" % (robot, r))
            if self.is_filtered(robot, r):
                stats['filtered'].append(r)
            else:
                stats['forwarded'].append(r)
                p.publish(data)
        self._stat_publisher.publish(json.dumps(stats))
    
    def _init_graph(self, topic, robots):
        self.graphs[topic] = {}
        for a in robots:
            self.graphs[topic][a] = {}  
            for b in robots:
                if a != b:
                    self.graphs[topic][a][b] = {"connected": True}               
        rospy.logdebug(self.graphs[topic])

    def _add_link(self, topic, src, tgt):       
        self.graphs[topic][src][tgt]["connected"] = True
        self.graphs[topic][tgt][src]["connected"] = True
    
    def _del_link(self, topic, src, tgt):
        self.graphs[topic][src][tgt]["connected"] = False
        self.graphs[topic][tgt][src]["connected"] = False
    
    def _publish_graph(self, topic):
        self._graph_publishers[topic].publish(json.dumps(self.graphs[topic]))        

    def run(self):
        while not rospy.is_shutdown():
            for t, v in self._config.items():
                for a, b in all_pairs(v.keys()):
                    if a == b:
                        continue
                    if self.is_filtered(a, b) or self.is_filtered(b, a):
                        self._del_link(t, a, b)
                    else:
                        self._add_link(t, a, b)
                self._publish_graph(t)
            rospy.sleep(1)
        
if __name__ == "__main__":
    rospy.init_node("vnet", log_level=rospy.DEBUG)
    vnet = VNetRos()
    vnet.run()
