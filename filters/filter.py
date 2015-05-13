from .basefilter import BaseFilter
from .pose_filter import *

""" Create a filter from its name """
def create_filter(name, **kwargs):
    for cls in BaseFilter.__subclasses__():
        if cls._name == name:
            return cls(**kwargs)
    return None
