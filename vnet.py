from threading import RLock

from filters.filter import create_filter
from proxies.proxy import create_proxy

class VNet:
    def __init__(self):
        self.filters_table = {}
        self.filters_lock = RLock()
        self.filters = []

    def add_filter(self, src, tgt, filter, **kwargs):
        self.filters_lock.acquire()
        try:
            if not src in self.filters_table.keys():
                self.filters_table[src] = {}
            if not tgt in self.filters_table[src].keys():
                self.filters_table[src][tgt] = []
            f = create_filter(filter, **kwargs)
            try:
                i = self.filters.index(f)
                del f
                f = self.filters[i]
                f.update(**kwargs)
            except:
                pass
            self.filters_table[src][tgt] += [f]
            return True
        except:
            return False
        finally:
            self.filters_lock.release()

    def del_filter(self, src, tgt, index):
        self.filters_lock.acquire()
        try:
            del self.filters_table[src][tgt][index]
            return True
        except:
            return False
        finally:
            self.filters_lock.release()
            
    def list_filters(self, src, tgt):
        try:
            return self.filters_table[src][tgt]
        except:
            return []
        
    def list_all_filters(self):
        return self.filters_table
            
    def is_filtered(self, src, tgt):
        try:
            filters = self.filters_table[src][tgt]
        except:
            filters = []
        return any(map(lambda f: f(src, tgt), filters))