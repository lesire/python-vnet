from .baseproxy import BaseProxy

""" Create a proxy from its name """
def create_proxy(name, filter_table, **kwargs):
    for cls in BaseProxy.__subclasses__():
        if cls._name == name:
            return cls(filter_table, **kwargs)
    return None
