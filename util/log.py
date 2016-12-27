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

    def __init__(self):
        self._rules = []

    def filter(self, record):
        '''
        Implement Python `logging.Filter` interface.
        '''
        for (ns, lvl) in self._rules:
            if record.name.startswith(ns):
                if record.levelno >= lvl:
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
