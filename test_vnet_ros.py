import rospy
import rosnode
import json
from vnet_ros import VNetRos
from std_msgs.msg import String

rospy.init_node("test_vnet_ros", log_level=rospy.DEBUG)

def receive_cb(data, r):
    rospy.loginfo("Reveiced %s on robot %s" % (str(data), str(r)))

vnet_config = {"test": {"r1": {"in": "/r1/test/in", "out": "/r1/test/out"}, \
                        "r2": {"in": "/r2/test/in", "out": "/r2/test/out"}, \
                        "r3": {"in": "/r3/test/in", "out": "/r3/test/out"}}}
publishers = {}

for r, t in vnet_config["test"].items():
    rospy.Subscriber(t["in"], String, receive_cb, r)
    publishers[r] = rospy.Publisher(t["out"], String, queue_size=1)

rospy.set_param("vnet_config", vnet_config)

#while not rospy.is_shutdown() and not "vnet" in rosnode.get_node_names():
#    rospy.sleep(1)
add_filter = rospy.Publisher("/vnet/add", String)
del_filter = rospy.Publisher("/vnet/del", String)

rospy.sleep(3)
rospy.loginfo("Send a message from r1 - everyone should receive")
publishers["r1"].publish("hello")

rospy.sleep(3)
rospy.loginfo("Block messages from r1 to r2")
add_filter.publish(json.dumps({"src": "r1", "tgt": "r2", "filter": "block"}))
rospy.sleep(3)
rospy.loginfo("Send a message from r1 - only r3 should receive")
publishers["r1"].publish("hello")
rospy.sleep(3)
rospy.loginfo("The other way?")
publishers["r2"].publish("hello")

rospy.sleep(3)
rospy.loginfo("Pass messages from r2 to r1")
del_filter.publish(json.dumps({"src": "r1", "tgt": "r2", "filter": "block"}))
rospy.sleep(3)
rospy.loginfo("Send a message from r1 - only r3 should receive")
publishers["r1"].publish("hello")
rospy.sleep(3)
rospy.loginfo("The other way?")
publishers["r2"].publish("hello")

rospy.spin()
