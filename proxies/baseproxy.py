from abc import ABCMeta, abstractmethod

""" Proxy meta class """
class ProxyMetaClass(ABCMeta):
    @property
    def name(self):
        return self._name

""" Base proxy class """
class BaseProxy:
    __metaclass__ = ProxyMetaClass

    def __init__(self, table):
        self._table = table

    @abstractmethod
    def read(self):
        return None

    @abstractmethod
    def send(self, destination, message):
        return True

    @abstractmehod
    def acknowledge(self, destination, ack):
        pass

    @abstractmethod
    def destinations(self, message):
        return None, None

    def run(self):
        msg = self.read()
        if msg:
            src, tgt = self.destinations(msg)
            filters = self._table[src][tgt]
            if all(map(lambda f: f(src, tgt), filters)):
                ack = self.send(tgt, msg)
                self.acknowledge(src, ack)
            else:
                self.acknowledge(src, False)
