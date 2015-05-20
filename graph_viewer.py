import rospy
from std_msgs.msg import String
import json
import igraph

class GraphViewer:
    def __init__(self):
        # Input graph
        self.sub = rospy.Subscriber("/vnet/graph/test", String, self._write_graph)

    def _write_graph(self, data):
        d = json.loads(data.data)
        rospy.logdebug(list(d.keys()))
        g = igraph.Graph(len(d))
        g.to_undirected()
        g.vs["name"] = list(d.keys())
        # Add edge information
        for src, dest in d.items():
            for tgt, conn in dest.items():
                if conn.get("connected", False):
                    g.add_edge(src, tgt)
        rospy.logdebug(str(g))        
        g.write_svg("/tmp/" + self.sub.name.replace('/', '_') + ".svg", layout=g.layout_circle(), labels="name")
               
if __name__ == "__main__":
    rospy.init_node("graph_viewer", log_level=rospy.DEBUG)
    gv = GraphViewer()
    rospy.spin()
