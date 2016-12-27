# from core import StoreInterface


# class Memory(object):
#     '''A read-only moment.'''
#     pass


# class Thought(object):
#     '''A writeable moment that provides .'''
#     pass

# class Pensive(object):

#     default_address = 'localhost:1234'

#     def __init__(self, address=None):
#         if not address:
#             address = self.default_address

#     def now(self):
#         '''Get writeable interface to the current database.'''

#         return StoreInterface()

#     def moment(self, tag):
#         '''Get a read-only interface to a prior database instant.'''
#         return StoreInterface()

#     @classmethod
#     def set_default(cls, address):
#         cls.default_address = address

from .server import PensiveServer
