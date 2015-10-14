#!/usr/bin/env python
import rospy
import rostopic
from std_msgs.msg import String,Empty

from vnet import VNet
import json
import time
from threading import RLock

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
                
        # Administration services
        rospy.Subscriber("/vnet/add", String, self._admin, self.add_filter)
        rospy.Subscriber("/vnet/del", String, self._admin, self.del_filter)
        rospy.Subscriber("/vnet/list", String, self._admin, self.list_filters)
        rospy.Subscriber("/vnet/listall", String, self._admin, self.list_all_filters)
        rospy.Subscriber("/vnet/reload", Empty, self._reload_config)
        
        # Statistics
        self._stat_publisher = rospy.Publisher("/vnet/statistics", String, queue_size=100)
        
        self._publishers = {}
        self._subscribers = {}
        self._graph_publishers = {}
        self.graphs = {}

        # Wait for all the topics to be published to have the topic class defined
        time.sleep(5)
        
        self._reload_config()

    def log(self, *args):
        rospy.loginfo(" ".join([str(s) for s in args]))

    def _reload_config(self, data=None):
        rospy.loginfo("Reloading vNet config")
        
        try:
            self._config = rospy.get_param("vnet/config")
            rospy.loginfo("Got configuration %s" % str(self._config))
        except:
            rospy.logwarn("No vNet configuration: vNet might do nothing!")
            self._config = {}
        
        # Remove obsolete topics publishers
        for channel in self._publishers.keys():
            if channel not in self._config:
                for robot,publisher in self._publishers[channel].items():
                    publisher.unregister()
                del self._publishers[channel]
                    
            for robot,publisher in self._publishers[channel].items():
                if robot not in self._config[channel]:
                    publisher.unregister()
                    
        # Remove obsolete channels subscribers
        for channel in self._subscribers.keys():
            if channel not in self._config:
                for robot,subscriber in self._subscribers[channel].items():
                    subscriber.unregister()
                del self._subscribers[channel]
                    
            for robot,subscriber in self._subscribers[channel].items():
                if robot not in self._config[channel]:
                    subscriber.unregister()
        
        self.channelLocks = {}
        for c in self._config:
            self.channelLocks[c] = RLock()
        
        self.robots = set()
        
        for channel,channel_info in self._config.items():
            # For each channel, get the message type
            msg_type = None
            well_defined = True
            for port in [d["in"] for d in channel_info.values()]:
                try:
                    t,_,_ = rostopic.get_topic_class(port)
                except:
                    continue #topic not yet defined
                if msg_type is not None and t is not None and t != msg_type:
                    rospy.logerr("Bad vNet config : the channel %s has conflicting message type : %s and %s" %(channel,msg_type,t))
                    well_defined = False
                elif msg_type is None and t is not None:
                    msg_type = t
            
            if msg_type is None:
                rospy.logerr("Cannot find a single defined topic for channel %s. Ignoring it" % channel)
                well_defined = False
            
            rospy.loginfo("Got type %s for %s channel" % (msg_type,channel))
            
            # Create new subscriber/publisher as needed
            if well_defined:
                if channel not in self._publishers:  self._publishers[channel] = {}
                if channel not in self._subscribers: self._subscribers[channel] = {}
                
                for robot,ports in channel_info.items():
                    if robot not in self._subscribers[channel]:
                        self._subscribers[channel][robot] = rospy.Subscriber(ports['out'], msg_type, self._forward, (robot, channel))
                    if robot not in self._publishers[channel]:
                        self._publishers[channel][robot] = rospy.Publisher(ports['in'], msg_type, queue_size=100)

                self._graph_publishers[channel] = rospy.Publisher("/vnet/graph/"+channel, String, queue_size=100)
                self._init_graph(channel, list(channel_info.keys()))

                self.robots = self.robots.union(set(channel_info.keys()))
                
            else:
                del self._config[channel]

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
        with self.channelLocks[topic]:
            rospy.logdebug("Received %s from %s on topic %s" % (str(data), robot, topic))
            rospy.logdebug(str(self.filters_table))
            stats = {'from': robot, 'topic': topic, 'filtered': [], 'forwarded': [], 'data': str(data)[:20], 'size': len(str(data))}
            for r, p in self._publishers[topic].items():
                if self.is_filtered(robot, r):
                    rospy.logdebug("Filter it from %s to %s" % (robot, r))
                    stats['filtered'].append(r)
                else:
                    rospy.logdebug("Forward it from %s to %s on %s" % (robot, r, p.name))
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
