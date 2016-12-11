'''
Core database components of Pensive.
'''

import logging
logger = logging.getLogger(__name__)

import re

class BoundKey(object):
    '''
    Encapsulates operations on a specific dictionary key.
    '''

    def __init__(self, obj, key):
        self._obj = obj
        self._key = key

    def set(self, value):
        '''Set the value.'''
        self._obj[self._key] = value

    def get(self):
        '''Get the value.'''
        return self._obj[self._key]

    def delete(self):
        '''Delete the key.'''
        del self._obj[self._key]

class MemoryStore(object):
    '''
    An in-memory hierarchical key-value database storing Python objects.
    '''

    _separator = re.compile(r'(?<!\\)/')

    def __init__(self, initial=None):
        '''
        Construct a key-value database with initial contents `initial`.
        '''

        self._root = {}
        if initial:
            self._root.update(initial)

    def put(self, key, value):
        '''
        Put `value` in the database with path `key`.
        '''

        logger.info('put: "{}"'.format(key))
        self._walk(key).set(value)

    def get(self, key):
        '''
        Get the value from the database with path `key`.
        '''

        logger.info('get: "{}"'.format(key))
        return self._walk(key).get()

    def delete(self, key):
        '''
        Delete the value from the database with path `key`.
        '''

        logger.info('delete: "{}"'.format(key))
        self._walk(key).delete()

    def find(self, key=None):
        '''
        List all keys in the database staring with `key`.

        If `key` is `None`, the entire database is considered.
        '''

        logger.info('find: "{}"'.format(key))

        if key:
            node = self._walk(key)
        else:
            node = self._root

        return self._keys(node)

    def _keys(self, node):
        '''
        Make a list of keys starting with `node`.

        Keys are returned as a list of tuples of key and subkeys.
        '''

        tree = []

        try:
            subkeys = node.keys()
        except AttributeError:
            return None
        else:
            for k in subkeys:
                # recurse down the tree
                tree.append((k, self._keys(node[k])))
            return tree


    def _walk(self, path, base=None):
        '''
        Walk through the key-value store to find the node referenced
        by the iterable `path` starting at `base`.

        If `path` is a string, it is separated into a list. If the path
        does not exist, it is created. If the path terminates prematurely
        with a value, `KeyError` is raised.
        '''

        if isinstance(path, basestring):
            # split up the path into a list of keys
            parts = self._separator.split(path)
            logger.debug('split path: "{}" -> {}'.format(path, parts))

            return self._walk(parts)

        else:
            # start walking from the base or the storage root
            node = base or self._root

            # iterate over all but the last key
            for (i, k) in enumerate(path[:-1]):
                try:
                    node = node[k]
                except KeyError:
                    # no key so make it
                    node[k] = {}
                    node = node[k]
                except Exception:
                    # dictionary access not available
                    logger.warn('path {} stops at {}'.format(path, path[:i]))
                    raise KeyError(path)

            # make a bound lookup for the final key
            return BoundKey(node, path[-1])
