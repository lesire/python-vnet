from basefilter import BaseFilter

""" Create a filter from its name """
def create_filter(name, **kwargs):
    for cls in BaseFilter.__subclasses__():
        if cls.name == name:
            return cls(**kwargs)
    return None
