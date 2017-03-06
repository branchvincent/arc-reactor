'''
Web client interface to `PensiveServer`.
'''

import logging

import httplib

from os import environ

from tornado.escape import to_basestring
from tornado.httpclient import HTTPClient
from tornado.httputil import url_concat

import json
import jsonschema

from .core import Store, StoreInterface
from . import DEFAULT_PORT

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

JSON_ENCODERS = {}
JSON_DECODERS = {}

def _json_encoder(obj):
    try:
        encode = JSON_ENCODERS[type(obj)]
    except KeyError:
        raise TypeError('cannot serialize {}'.format(type(obj)))
    else:
        return encode(obj)

def _json_decoder(obj):
    if not isinstance(obj, dict):
        return obj

    for k in obj.keys():
        if k in JSON_DECODERS:
            return JSON_DECODERS[k](obj[k])

    return obj

def json_encode(obj, **kwargs):
    kwargs['default'] = _json_encoder
    return json.dumps(obj, **kwargs).replace("</", "<\\/")

def json_decode(data, **kwargs):
    kwargs['object_hook'] = _json_decoder
    return json.loads(to_basestring(data), **kwargs)

class JSONClientMixin(object):
    '''
    Internal convenience class for sending/receiving JSON over HTTP.
    '''

    def __init__(self, base_url, client=None):
        if not base_url.startswith('http://'):
            base_url = 'http://' + base_url

        self._base_url = base_url.rstrip('/') + '/'
        self._client = client or HTTPClient()

    def _fetch(self, path, method, body=None, args=None, schema=None):
        '''
        Helper for HTTP requests.

        If `body` is a string, it is sent as is. If `body is not None`,
        it is JSON-encoded and sent.

        If `schema is not None`, it is used to validate any response
        body for JSON decoding. If `schema` is not provided, the response
        body is ignored.
        '''

        # build the complete URL
        path = self._base_url + path

        # encode object body using JSON
        if body and not isinstance(body, basestring):
            # see tornado.escape.json_encode()
            body = json_encode(body)

        # encode the query parameters
        if args:
            if not isinstance(args, basestring):
                args = dict([(k, json_encode(v)) for (k, v) in args.iteritems()])
            path = url_concat(path, args)

        # perform the request
        response = self._client.fetch(path,
                                      method=method,
                                      body=body,
                                      headers={'Accept': 'application/json'})

        logger.debug('{} {} -> {}'.format(method, path, response.code))

        # map common HTTP errors to exceptions
        if response.code == 404:
            raise KeyError(path)
        elif response.code not in [httplib.OK, httplib.CREATED, httplib.NO_CONTENT]:
            logger.error('unexpected HTTP response: {} {}\
                \n\nResponse:\n{}'.format(response.code,
                                          response.reason,
                                          response.body))
            raise RuntimeError('{} {}'.format(response.code, response.reason))

        # decode the response using JSON if a schema is provided
        if schema:
            if response.code == httplib.NO_CONTENT:
                raise RuntimeError('server indicated no content when expecting response body')

            try:
                # see tornado.escape.json_decode()
                obj = json_decode(response.body)
                jsonschema.validate(obj, schema)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.error('malformed response: {}\
                    \n\nResponse:\n{}'.format(exc, response.body))
                raise RuntimeError('malformed response')
            else:
                return obj

class BatchStoreInterface(StoreInterface):
    '''
    Batch operation interface for `PensiveServer`.
    '''

    def multi_get(self, keys, root=None):
        raise NotImplementedError

    def multi_put(self, mapping, root=None):
        raise NotImplementedError

    def multi_delete(self, keys, root=None):
        raise NotImplementedError

class StoreProxy(JSONClientMixin, BatchStoreInterface):
    '''
    Proxy for a `Store` served over HTTP.
    '''

    GET_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
            'value': {}
        },
        'required': ['value'],
    }

    MULTI_GET_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
    }

    def __init__(self, host, instance=None, **kwargs):
        if instance is None:
            base_url = '{}/d/'.format(host.rstrip('/'))
        else:
            base_url = '{}/i/{}/'.format(host.rstrip('/'), instance)

        super(StoreProxy, self).__init__(base_url, **kwargs)

    def _concat_key(self, key):
        if key and not isinstance(key, basestring):
            key = Store.SEPARATOR.join(key)

        return key

    def get(self, key=None, default=None, strict=False):
        '''
        Call `get()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        result = self._fetch(key or '', 'GET', args={'strict': strict}, schema=StoreProxy.GET_SCHEMA)['value']
        # optimize out sending default over the network
        if result is None:
            return default
        else:
            return result

    def multi_get(self, keys, root=None):
        '''
        Perform a batch `get()` on the remote `Store`
        in a single HTTP request.
        '''

        keys = [self._concat_key(key) for key in keys]
        return self._fetch(root or '', 'POST',
                           body={'operation': 'GET', 'keys': keys},
                           schema=StoreProxy.MULTI_GET_SCHEMA)

    def put(self, key=None, value=None, strict=False):
        '''
        Call `put()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        return self._fetch(key or '', 'PUT', args={'strict': strict}, body={'value': value})

    def multi_put(self, mapping, root=None):
        '''
        Perform a batch `put()` on the remote `Store`
        in a single HTTP request.
        '''

        mapping = {self._concat_key(key): value for (key, value) in mapping.iteritems()}
        return self._fetch(root or '', 'PUT', body={'keys': mapping})

    def delete(self, key=None, strict=False):
        '''
        Call `delete()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        return self._fetch(key or '', 'DELETE', args={'strict': strict})

    def multi_delete(self, keys, root=None):
        '''
        Perform a batch `delete()` on the remote `Store`
        in a single HTTP request.
        '''

        keys = [self._concat_key(key) for key in keys]
        return self._fetch(root or '', 'POST',
                           body={'operation': 'DELETE', 'keys': keys})

class StoreTransaction(StoreInterface):
    '''
    Helper class for building a batch modification to a `Store`.

    The `StoreTransaction` acts as a changelog for puts and deletes for
    a store. All puts and deletes are buffered until `commit()` is called,
    which flushes the changelog to the destination which must implement
    `BatchStoreInterface`. All puts and deletes on the `StoreTransaction`
    are withheld until committed.
    '''

    def __init__(self, source=None):
        '''
        Initialize a `StoreTransaction`.

        If `source`, all lookups into the transaction will be routed
        to `source` if the given key has not been modified by the
        transaction. This allows the `StoreTransaction` to work as a drop-in
        replacement for any object implementing `StoreInterface`.
        '''

        self._source = source
        self.reset()

    def get(self, key=None):
        '''
        Perform a lookup in the `StoreTransaction`.

        If `key` is set in the cached transaction changes, the lookup
        will be into the changes. Otherwise, the lookup will pass through
        to the transaction source if one is provided.
        '''

        try:
            # attempt lookup from transaction
            return self._trans.get(key, strict=True)
        except KeyError:
            pass

        # attempt lookup from source
        if self._source:
            return self._source.get(key)

        # default
        return None

    def put(self, key=None, value=None):
        '''
        Record a `put()` in the transaction.
        '''

        self._trans.put(key, value)

    def delete(self, key=None):
        '''
        Record a `delete()` in the transaction.
        '''

        self._trans.delete(key)

    def commit(self, destination):
        '''
        Flush all puts and deletes from in this transaction to
        `destination`, which must implement `BatchStoreInterface`.
        '''

        destination.multi_put(self._trans.flatten(strict=True))

    def reset(self):
        '''
        Clear all recorded puts and deletes from the transaction.
        '''

        self._trans = Store()

class PensiveClient(JSONClientMixin):
    '''
    Client for accessing and manipulating `Store`s on a PensiveServer.
    '''

    GET_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
            'index': {
                'type': 'array',
                'items': {
                    'type': ['string', 'null'],
                },
                'uniqueItems': True
            }
        },
        'required': ['index'],
    }

    DEFAULT_STORE = object()

    def __init__(self, host=None, **kwargs):
        self._host = host
        if not self._host:
            # fall back to environment variable
            self._host = environ.get('PENSIVE_SERVER', None)
        if not self._host:
            # fall back to default
            self._host = 'http://localhost:{}/'.format(DEFAULT_PORT)

        super(PensiveClient, self).__init__(self._host, **kwargs)

    def default(self):
        '''
        Convenience method for explicitly accessing default store.
        '''

        return self.store(None)

    def store(self, instance=None):
        '''
        Get a `StoreProxy` for the requested instance.

        Checks the existence of `instance` first but offers not guarantees
        about race conditions.
        '''

        if instance not in self.index():
            raise KeyError(instance)

        return StoreProxy(self._host, instance, client=self._client)

    def index(self):
        '''
        Return the list of all known `Store` instances.
        '''

        return self._fetch('s', 'GET', schema=PensiveClient.GET_SCHEMA)['index']

    def create(self, instance, parent=None, force=False):
        '''
        Creates a new instance and returns the corresponding `StoreProxy`.

        If `instance` already exists and not `force`, `ValueError` is raised.
        Otherwise, the existing instance is first deleted.

        If `parent is PensiveClient.DEFAULT_STORE`, an empty instance is created. Otherwise,
        the created instance is a fork of `parent`.
        '''

        if instance in self.index():
            if force:
                self.delete(instance)
            else:
                raise ValueError('instance already exists')

        if parent is PensiveClient.DEFAULT_STORE:
            self._fetch('s/{}'.format(instance), 'PUT', body={'parent': None})
        elif parent is None:
            self._fetch('s/{}'.format(instance), 'PUT', body='')
        else:
            self._fetch('s/{}'.format(instance), 'PUT', body={'parent': parent})

        return self.store(instance)

    def delete(self, instance):
        '''
        Deletes the store `instance`.
        '''

        return self._fetch('s/{}'.format(instance), 'DELETE')
