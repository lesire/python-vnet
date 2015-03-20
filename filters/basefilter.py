from abc import ABCMeta, abstractmethod

""" Filter meta class """
class FilterMetaClass(ABCMeta):
    @property
    def name(self):
        return self._name

""" Base filter class """
class BaseFilter:
    __metaclass__ = FilterMetaClass    

    def __call__(self, source, target):
        return self.filtered(source, target)

    @abstractmethod
    def filtered(self, source, target):
        return None

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
