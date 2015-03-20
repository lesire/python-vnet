from baseproxy import BaseProxy

""" Create a proxy from its name """
def create_proxy(name, filter_table, **kwargs):
    for cls in ProxyFilter.__subclasses__():
        if cls.name == name:
            return cls(filter_table, **kwargs)
    return None
