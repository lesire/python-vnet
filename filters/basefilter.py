from abc import ABCMeta, abstractmethod

""" Base filter class """
class BaseFilter:
    __metaclass__ = ABCMeta    

    def __call__(self, source, target):
        return self.filtered(source, target)

    @abstractmethod
    def filtered(self, source, target):
        return None
    
    def __str__(self):
        return self._name

""" Pass filter: every message is forwarded """
class PassFilter(BaseFilter):
    _name = "pass"

    def filtered(self, source, target):
        return True

""" Block filter: every message is blocked """
class BlockFilter(BaseFilter):
    _name = "block"

    def filtered(self, source, target):
        return False
