from threading import RLock

from filters.filter import create_filter
from proxies.proxy import create_proxy

import traceback

class VNet:
    def __init__(self):
        self.filters_table = {}
        self.filters_lock = RLock()
        self.filters = []

        self.robots = set() # list of all know robots

    def add_filter(self, src, tgt, filter, **kwargs):
        self.filters_lock.acquire()

        try:
            if src == "*":
                for r in self.robots:
                    if r != tgt:
                        self.add_filter(r, tgt, filter, **kwargs)
                return True

            elif tgt == "*":
                for r in self.robots:
                    if r != src:
                        self.add_filter(src, r, filter, **kwargs)
                return True

            elif "bidir" in kwargs and kwargs["bidir"]:
                del kwargs["bidir"]
                self.add_filter(src, tgt, filter, **kwargs)
                self.add_filter(tgt, src, filter, **kwargs)
                return True

            print("Adding filter %s/%s: %s (%s)" % (src, tgt, filter, str(kwargs)))

            if not src in self.filters_table.keys():
                self.filters_table[src] = {}
            if not tgt in self.filters_table[src].keys():
                self.filters_table[src][tgt] = []
            f = create_filter(filter, src=src, tgt=tgt, **kwargs)
            try:
                i = self.filters.index(f)
                del f
                f = self.filters[i]
                f.update(**kwargs)
            except:
                pass
            self.filters_table[src][tgt] += [f]
            return True
        except Exception as e:
            print(traceback.format_exc())
            return False
        finally:
            self.filters_lock.release()

    def del_filter(self, src, tgt, index=-1, filter=None, **kwargs):
        self.filters_lock.acquire()
        print("Del %s %s %s %s %s" % (src, tgt, index, filter, kwargs))
        try:
            if src == "*":
                for r in self.robots:
                    if r != tgt:
                        self.del_filter(r, tgt, index=index, filter=filter, **kwargs)
                return True

            elif tgt == "*":
                for r in self.robots:
                    if r != src:
                        self.del_filter(src, r, index=index, filter=filter, **kwargs)
                return True

            elif "bidir" in kwargs and kwargs["bidir"]:
                del kwargs["bidir"]
                self.del_filter(src, tgt, index=index, filter=filter, **kwargs)
                self.del_filter(tgt, src, index=index, filter=filter, **kwargs)
                return True

            elif index == "*":
                for i in reversed(range(len(self.filters_table[src][tgt]))):
                    self.del_filter(src, tgt, index=i, filter=None, **kwargs)
                return True

            print("Deleting filter %s/%s: %s" % (src, tgt, (str(index) if filter is None else filter)))

            if filter is None:
                del self.filters_table[src][tgt][index]
            else:
                for i in range(len(self.filters_table[src][tgt])):
                    f = self.filters_table[src][tgt][i]
                    if f._name == filter:
                        return self.del_filter(src, tgt, i, None)
            return True
        except Exception as e:
            print(traceback.format_exc())
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

    def is_filtered_directed(self, src, tgt):
        try:
            filters = self.filters_table[src][tgt]
        except:
            filters = []
        return any(map(lambda f: f(src, tgt), filters))
            
    def is_filtered(self, src, tgt):
        return self.is_filtered_directed(src, tgt) or self.is_filtered_directed(tgt, src)
