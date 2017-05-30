'''
Web client interface to `PensiveServer`.
'''

import logging

import http.client

from os import environ

from tornado.httpclient import AsyncHTTPClient
from tornado.httputil import url_concat
from tornado.gen import coroutine, Return

import jsonschema

from past.builtins import basestring
from future.utils import viewitems

from .core import Store
from .client import BatchStoreInterface, StoreProxy, PensiveClient, json_encode, json_decode
from . import DEFAULT_PORT

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

# TODO: reduce code duplication with pensive.client

class JSONClientMixinAsync(object):
    '''
    Internal convenience class for sending/receiving JSON over HTTP.
    '''

    def __init__(self, base_url, client=None):
        if not base_url.startswith('http://'):
            base_url = 'http://' + base_url

        self._base_url = base_url.rstrip('/') + '/'
        self._client = client or AsyncHTTPClient()

    @coroutine
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
                args = dict([(k, json_encode(v)) for (k, v) in viewitems(args)])
            path = url_concat(path, args)

        # perform the request
        response = yield self._client.fetch(path,
                                            method=method,
                                            body=body,
                                            headers={'Accept': 'application/json'})

        logger.debug('{} {} -> {}'.format(method, path, response.code))

        # map common HTTP errors to exceptions
        if response.code == 404:
            raise KeyError(path)
        elif response.code not in [http.client.OK, http.client.CREATED, http.client.NO_CONTENT]:
            logger.error('unexpected HTTP response: {} {}\
                \n\nResponse:\n{}'.format(response.code,
                                          response.reason,
                                          response.body))
            raise RuntimeError('{} {}'.format(response.code, response.reason))

        # decode the response using JSON if a schema is provided
        if schema:
            if response.code == http.client.NO_CONTENT:
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
                raise Return(obj)

class StoreProxyAsync(JSONClientMixinAsync, BatchStoreInterface):
    '''
    Proxy for a `Store` served over HTTP.
    '''

    def __init__(self, host, instance=None, **kwargs):
        if instance is None:
            base_url = '{}/d/'.format(host.rstrip('/'))
        else:
            base_url = '{}/i/{}/'.format(host.rstrip('/'), instance)

        super(StoreProxyAsync, self).__init__(base_url, **kwargs)

    def _concat_key(self, key):
        if key and not isinstance(key, basestring):
            key = Store.SEPARATOR.join(key)

        return key

    @coroutine
    def get(self, key=None, default=None, strict=False):
        '''
        Call `get()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        result = yield self._fetch(key or '', 'GET', args={'strict': strict}, schema=StoreProxy.GET_SCHEMA)
        # optimize out sending default over the network
        if result['value'] is None:
            raise Return(default)
        else:
            raise Return(result['value'])

    @coroutine
    def multi_get(self, keys, root=None):
        '''
        Perform a batch `get()` on the remote `Store`
        in a single HTTP request.
        '''

        keys = [self._concat_key(key) for key in keys]
        result = yield self._fetch(root or '', 'POST',
                                   body={'operation': 'GET', 'keys': keys},
                                   schema=StoreProxy.MULTI_GET_SCHEMA)
        raise Return(result)

    @coroutine
    def put(self, key=None, value=None, strict=False):
        '''
        Call `put()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        result = yield self._fetch(key or '', 'PUT', args={'strict': strict}, body={'value': value})
        raise Return(result)

    @coroutine
    def multi_put(self, mapping, root=None):
        '''
        Perform a batch `put()` on the remote `Store`
        in a single HTTP request.
        '''

        mapping = {self._concat_key(key): value for (key, value) in viewitems(mapping)}
        result = yield self._fetch(root or '', 'PUT', body={'keys': mapping})
        raise Return(result)

    @coroutine
    def delete(self, key=None, strict=False):
        '''
        Call `delete()` on the remote `Store`.
        '''

        key = self._concat_key(key)
        result = yield self._fetch(key or '', 'DELETE', args={'strict': strict})
        raise Return(result)

    @coroutine
    def multi_delete(self, keys, root=None):
        '''
        Perform a batch `delete()` on the remote `Store`
        in a single HTTP request.
        '''

        keys = [self._concat_key(key) for key in keys]
        result = yield self._fetch(root or '', 'POST',
                                   body={'operation': 'DELETE', 'keys': keys})
        raise Return(result)

class PensiveClientAsync(JSONClientMixinAsync):
    '''
    Client for accessing and manipulating `Store`s on a PensiveServer.
    '''

    def __init__(self, host=None, **kwargs):
        self._host = host
        if not self._host:
            # fall back to environment variable
            self._host = environ.get('PENSIVE_SERVER', None)
        if not self._host:
            # call back to default
            self._host = 'http://localhost:{}/'.format(DEFAULT_PORT)

        super(PensiveClientAsync, self).__init__(self._host, **kwargs)

    @coroutine
    def default(self):
        '''
        Convenience method for explicitly accessing default store.
        '''

        result = yield self.store(None)
        raise Return(result)


    @coroutine
    def store(self, instance=None):
        '''
        Get a `StoreProxy` for the requested instance.

        Checks the existence of `instance` first but offers not guarantees
        about race conditions.
        '''

        index = yield self.index()
        if instance not in index:
            raise KeyError(instance)

        raise Return(StoreProxyAsync(self._host, instance, client=self._client))

    @coroutine
    def index(self):
        '''
        Return the list of all known `Store` instances.
        '''

        result = yield self._fetch('s', 'GET', schema=PensiveClient.GET_SCHEMA)
        raise Return(result['index'])

    @coroutine
    def create(self, instance, parent=None, force=False):
        '''
        Creates a new instance and returns the corresponding `StoreProxy`.

        If `instance` already exists and not `force`, `ValueError` is raised.
        Otherwise, the existing instance is first deleted.

        If `parent is PensiveClient.DEFAULT_STORE`, an empty instance is created. Otherwise,
        the created instance is a fork of `parent`.
        '''

        index = yield self.index()
        if instance in index:
            if force:
                yield self.delete(instance)
            else:
                raise ValueError('instance already exists')

        if parent is PensiveClient.DEFAULT_STORE:
            yield self._fetch('s/{}'.format(instance), 'PUT', body={'parent': None})
        elif parent is None:
            yield self._fetch('s/{}'.format(instance), 'PUT', body='')
        else:
            yield self._fetch('s/{}'.format(instance), 'PUT', body={'parent': parent})

        raise Return(self.store(instance))

    @coroutine
    def delete(self, instance):
        '''
        Deletes the store `instance`.
        '''

        result = self._fetch('s/{}'.format(instance), 'DELETE')
        raise Return(result)
