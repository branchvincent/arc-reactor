
from tornado.escape import json_encode, json_decode
from tornado.httpclient import HTTPClient

from pensive.core import Store

import jsonschema

import logging
logger = logging.getLogger(__name__)

class StoreInterface(object):
    '''
    Basic interface for a `Store`.
    '''

    def get(self, path):
        raise NotImplementedError

    def put(self, key, path):
        raise NotImplementedError

    def delete(self, key):
        raise NotImplementedError

class BatchStoreInterface(StoreInterface):
    '''
    Batch operation interface for `PensiveServer`.
    '''

    def multi_get(self, paths, root=None):
        raise NotImplementedError

    def multi_put(self, mapping, root=None):
        raise NotImplementedError

    def multi_delete(self, paths, root=None):
        raise NotImplementedError

class StoreProxy(BatchStoreInterface):
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

    def __init__(self, host, instance=None, client=None):
        self._host = host
        self._instance = instance

        if instance is None:
            self._base_url = '{}/d/'.format(host)
        else:
            self._base_url = '{}/i/{}/'.format(host, instance)
        if not self._base_url.startswith('http://'):
            self._base_url = 'http://' + self._base_url

        self._client = client or HTTPClient()

    def _fetch(self, path, method, body=None, schema=None):
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
            body = json_encode(body)

        # perform the request
        response = self._client.fetch(path,
                                      method=method,
                                      body=body,
                                      headers={'Accept': 'application/json'})

        logger.debug('{} {} -> {}'.format(method, path, response.code))

        # map common HTTP errors to exceptions
        if response.code == 404:
            raise KeyError(path)
        elif response.code != 200:
            logger.error('unexpected HTTP response: {} {} \
                \n\nResponse:\n{}'.format(response.code,
                                          response.reason,
                                          response.body))
            raise RuntimeError('{} {}'.format(response.code, response.reason))

        # decode the response using JSON if a schema is provided
        if schema:
            try:
                obj = json_decode(response.body)
                jsonschema.validate(obj, schema)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.error('malformed response: {} \
                    \n\nResponse:\n{}'.format(exc, response.body))
                raise RuntimeError('malformed response')
            else:
                return obj

    def get(self, path):
        '''
        Call `get()` on the remote `Store`.
        '''

        return self._fetch(path, 'GET', schema=StoreProxy.GET_SCHEMA)['value']

    def multi_get(self, paths, root=None):
        '''
        Perform a batch `get()` on the remote `Store`
        in a single HTTP request.
        '''

        return self._fetch(root or '', 'POST',
                           body={'operation': 'GET', 'keys': paths},
                           schema=StoreProxy.MULTI_GET_SCHEMA)


    def put(self, path, value):
        '''
        Call `put()` on the remote `Store`.
        '''

        return self._fetch(path, 'PUT', body={'value': value})

    def multi_put(self, mapping, root=None):
        '''
        Perform a batch `put()` on the remote `Store`
        in a single HTTP request.
        '''

        return self._fetch(root or '', 'PUT', body={'keys': mapping})

    def delete(self, path):
        '''
        Call `delete()` on the remote `Store`.
        '''

        return self._fetch(path, 'DELETE')

    def multi_delete(self, paths, root=None):
        '''
        Perform a batch `delete()` on the remote `Store`
        in a single HTTP request.
        '''

        return self._fetch(root or '', 'POST',
                           body={'operation': 'DELETE', 'keys': paths})

class Transaction(StoreInterface):
    def __init__(self, base=None):
        self._base = base

    def commit(self):
        pass

class PensiveClient(object):
    pass
