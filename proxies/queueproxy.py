from baseproxy import BaseProxy
from Queue import Queue, Empty

""" Queue proxy class """
class QueueProxy(BaseProxy):
    _name = "queue"

    def __init__(self, table, queue):
        BaseProxy.__init__(self, table)
        self.msg_queue = queue

    def read(self):
        try:
            return self.msg_queue.get(False)
        except Empty:
            return None

    def send(self, destination, message):
        print(message)
        return True

    def acknowledge(self, destination, ack):
        print(ack)

    def destinations(self, message):
        return None, None

