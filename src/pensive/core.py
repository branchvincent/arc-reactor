'''
Core database components of Pensive.
'''

# pylint: disable=protected-access

import re

import logging
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class Store(object):
    r'''
    Hierarchical, recursive key-value database for storing arbitrary objects.

    Objects are stored with hierarchical keys like file paths.
    Each key level is delimited by a forward slash (`/`).
    The delimiter can be escaped with a preceding backslash (`\`).

    For example, a key might look like `some/data/here`.
    The data can be accessed at the key `some`, `some/data`, or
    `some/data/here`. Getting data an a non-terminal level will
    return a dictionary of all keys and values below the requested
    key.

    Values that equate to `False` are dropped form the database and
    will thus not appear in results. Consequently, a key in the
    database can be deleted by setting it to `None`.
    '''

    SEPARATOR = '/'
    _separator = re.compile(r'(?<!\\)' + SEPARATOR)

    def __init__(self, data=None):
        if data is not None:
            self._deserialize(data)

            self._needs_cull = True
        else:
            self._value = None
            self._children = None

    def get(self, key=None, strict=False):
        '''
        Get the value referenced by `key` from the store.

        If `strict`, `KeyError` will be raised for an unknown `key`.
        '''

        logger.debug('get: "{}"'.format(key))
        return self._get(key, strict)

    def _get(self, key, strict):
        if not key:
            # null key indicates the value of this Store
            return self._serialize()
        else:
            # split the key into parts if it is a string path
            if isinstance(key, basestring):
                key = self._separator.split(key)

            # get of store without children is null
            if not self._children:
                if strict:
                    raise KeyError
                return None

            try:
                # recurse
                return self._children[key.pop(0)]._get(key, strict)
            except KeyError:
                # no such child
                if strict:
                    raise
                return None

    def put(self, key=None, value=None):
        '''
        Put `value` into the store.

        If `key is None`, the store takes the value `value`.
        This erases all children. If `key is not None`, a
        substore is created and `put` is called recursively.

        If `value is None`, the store referenced by `key` is
        deleted.
        '''

        logger.debug('put: "{}" -> {}'.format(key, value))
        return self._put(key, value)

    def _put(self, key, value):
        if not key:
            self._deserialize(value)
        else:
            # split the key into parts if it is a string path
            if isinstance(key, basestring):
                key = self._separator.split(key)

            # initialize the children map
            if not self._children:
                self._children = {}
                self._value = None

            # create a child if needed and recurse
            self._children.setdefault(key.pop(0), Store())._put(key, value)

    def delete(self, key=None):
        '''
        Convenience function for deletion.  See `put`.
        '''

        self.put(key, None)

    def index(self, key=None):
        '''
        Get an index of the store starting with `key`.

        If `key` does not exist, the index is `None`.
        If `key` does exist, the index is a dictionary of
        dictionaries of keys terminated by dictionary keys
        with `{}` values.
        '''

        logger.debug('index: "{}"'.format(key))
        return self._index(key)

    def _index(self, key):
        if not key:
            if self._children:
                # recursively serialize
                result = [(k, c.index()) for (k, c) \
                    in self._children.iteritems()]
                # remove null values
                return {k: v for (k, v) in result if v is not None}
            elif self._value is not None:
                # an empty dictionary indicates a value
                return {}
            else:
                return None
        else:
            # split the key into parts if it is a string path
            if isinstance(key, basestring):
                key = self._separator.split(key)

            # index of store without children is null
            if not self._children:
                return None

            try:
                # recurse
                return self._children[key.pop(0)]._index(key)
            except KeyError:
                # no such child
                return None

    def cull(self):
        '''Clean out null keys from the database.'''

        logger.debug('cull')
        return self._cull()

    def _cull(self):
        if self._children:
            keys = self._children.keys()

            for k in keys:
                # cull all children
                if self._children[k]._cull():
                    del self._children[k]

            # check if this store should be culled
            return len(self._children) == 0

        else:
            # cull null values
            return self._value is None

    def fork(self):
        '''
        Make a shallow-copy of the `Store`.
        '''

        logger.debug('copy')
        return self._copy()

    def is_empty(self):
        '''
        Test if the `Store` contains only null data.
        '''

        if self._children:
            # check that there exists a non-empty child
            for child in self._children.values():
                if not child.is_empty():
                    return False

            return True
        else:
            # check if the stored value is non-null
            return self._value is None

    def flatten(self, strict=False):
        '''
        Return a dictionary all of values in the `Store`.

        The key is the complete path of the value.

        If `strict`, empty values are included.
        '''
        if self._children:
            result = {}
            # recursively flatten
            for (k, child) in self._children.iteritems():
                # prepend the path with the child key and skip over
                # empty values if not strict
                result.update({((k + self.SEPARATOR + p).strip('/'), v) \
                    for (p, v) in child.flatten(strict).iteritems() \
                    if strict or v is not None})
            return result
        else:
            return {'': self._value}

    def _serialize(self):
        '''
        Construct a dictionary representation of this `Store`.
        '''

        if self._children:
            # recursively serialize
            result = [(k, c._serialize()) for (k, c) \
                in self._children.iteritems()]
            # remove null values
            result = {k: v for (k, v) in result if v is not None}
            # check if non-null result
            if result:
                return result
            else:
                return None
        else:
            return self._value

    def _deserialize(self, data):
        '''
        Construct a `Store` from a dictionary representation.
        '''

        try:
            # check if there are subkeys or not
            keys = data.keys()
        except AttributeError:
            # set up a value Store
            self._value = data
            self._children = None
        else:
            # set up a subkey Store
            self._value = None
            self._children = {}
            # recurse through the subkeys
            for k in keys:
                self._children[k] = Store(data[k])

    def _copy(self):
        '''
        Make a shallow copy of `Store`.
        '''

        if self._children:
            # recursively copy
            result = [(k, c._copy()) for (k, c) \
                in self._children.iteritems()]
            # remove null values
            result = {k: v for (k, v) in result if v is not None}
            # check if non-null result
            if result:
                store = Store()
                store._children = result
                return store
            else:
                return None
        elif self._value is not None:
            store = Store()
            store._value = self._value
            return store
        else:
            return None
