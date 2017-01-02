'''
Utility classes and functions for logging.
'''

import logging

class LevelFilter(logging.Filter):
    '''
    Python logging filter to replicate `logging.Logger.setLevel()` functionality
    at the `logging.Handler` level.

    This is relevant because setting levels for loggers prevents
    filtered messages from reaching any handlers. Thus, all handlers
    are bound by the same level filters. Doing the filtering at the
    handler level allows each handler to have a separate level filtering
    scheme.
    '''

    def __init__(self, *args, **kwargs):
        super(LevelFilter, self).__init__(*args, **kwargs)
        self._rules = []

    def filter(self, record):
        '''
        Implement Python `logging.Filter` interface.
        '''
        for (namespace, level) in self._rules:
            if record.name.startswith(namespace):
                if record.levelno >= level:
                    return True

        return False

    def add(self, namespace, level):
        '''
        Add a new module namespace level filter.
        '''
        self.remove(namespace)
        self._rules.append((namespace, level))
        # keep the rules in reverse sorted order for easy filtering
        self._rules.sort(reverse=True)

    def remove(self, namespace):
        '''
        Remove a module namespace level filter.
        '''
        self._rules = [x for x in self._rules if x[0] != namespace]

# class NewStyleLogRecord(logging.LogRecord):
#     def getMessage(self):
#         """
#         Return the message for this LogRecord.

#         Return the message for this LogRecord after merging any user-supplied
#         arguments with the message.
#         """
#         msg = self.msg
#         if not isinstance(msg, basestring):
#             try:
#                 msg = str(self.msg)
#             except UnicodeError:
#                 msg = self.msg      #Defer encoding till later
#         if self.args:
#             msg = msg.format(*self.args)
#         return msg

# class NewStyleLogger(logging.Logger):
#     def makeRecord(self, name, level, fn, lno, msg, args, exc_info, func=None, extra=None):
#         """
#         A factory method which can be overridden in subclasses to create
#         specialized LogRecords.
#         """
#         rv = NewStyleLogRecord(name, level, fn, lno, msg, args, exc_info, func)
#         if extra is not None:
#             for key in extra:
#                 if (key in ["message", "asctime"]) or (key in rv.__dict__):
#                     raise KeyError("Attempt to overwrite %r in LogRecord" % key)
#                 rv.__dict__[key] = extra[key]
#         return rv

# logging.setLoggerClass(NewStyleLogger)
