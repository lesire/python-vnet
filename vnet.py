from threading import RLock

from filters.filter import create_filter
from proxies.proxy import create_proxy

class VNet:
    def __init__(self):
        self.filters_table = {}
        self.filters_lock = RLock()

    def add_filter(self, src, tgt, fname):
        self.filters_lock.acquire()
        try:
            if not src in self.filters_table.keys():
                self.filters_table[src] = {}
            if not tgt in self.filters_table[src].keys():
                self.filters_table[src][tgt] = []
            self.filters_table[src][tgt] += [create_filter(fname)]
        except:
            pass
        finally:
            self.filters_lock.release()
