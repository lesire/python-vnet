from abc import ABCMeta, abstractmethod
from threading import Thread
from time import sleep

""" Proxy meta class """
class ProxyMetaClass(ABCMeta):
    @property
    def name(self):
        return self._name

""" Base proxy class """
class BaseProxy(Thread):
    __metaclass__ = ProxyMetaClass

    def __init__(self, table):
        Thread.__init__(self)
        self._table = table
        self.running = True

    @abstractmethod
    def read(self):
        return None

    @abstractmethod
    def send(self, destination, message):
        return True

    @abstractmethod
    def acknowledge(self, destination, ack):
        pass

    @abstractmethod
    def destinations(self, message):
        return None, None

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            msg = self.read()
            if msg:
                src, tgt = self.destinations(msg)
                try:
                    filters = self._table[src][tgt]
                except:
                    filters = []
                if all(map(lambda f: f(src, tgt), filters)):
                    ack = self.send(tgt, msg)
                    self.acknowledge(src, ack)
                else:
                    self.acknowledge(src, False)
            sleep(.1)
