
from tornado.escape import json_encode, json_decode
from tornado.httpclient import HTTPClient

import jsonschema

import logging
logger = logging.getLogger(__name__)

class StoreInterface(object):
    def get(self, path):
        pass

    def put(self, key, path):
        pass

    def delete(self, key):
        pass

class StoreProxy(StoreInterface):
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
        path = self._base_url + path

        if body and not isinstance(body, basestring):
            body = json_encode(body)

        response = self._client.fetch(path,
                                      method=method,
                                      body=body,
                                      headers={'Accept': 'application/json'})

        logger.debug('{} {} -> {}'.format(method, path, response.code))

        if response.code == 404:
            raise KeyError(path)
        elif response.code == 400:
            raise RuntimeError(response.reason)

        if schema:
            try:
                obj = json_decode(response.body)
                jsonschema.validate(obj, schema)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.error('malformed response: {}\n\nResponse:\n{}'.format(exc, response.body))
                raise RuntimeError('malformed response')
            else:
                return obj

    def get(self, path):
        return self._fetch(path, 'GET', schema=StoreProxy.GET_SCHEMA)['value']

    def multi_get(self, paths, root=None):
        return self._fetch(root or '', 'POST',
                           body={'operation': 'GET', 'keys': paths},
                           schema=StoreProxy.MULTI_GET_SCHEMA)


    def put(self, path, value):
        return self._fetch(path, 'PUT', body={'value': value})

    def multi_put(self, mapping, root=None):
        return self._fetch(root or '', 'PUT', body={'keys': mapping})

    def delete(self, path):
        return self._fetch(path, 'DELETE')

    def multi_delete(self, paths, root=None):
        return self._fetch(root or '', 'POST',
                           body={'operation': 'DELETE', 'keys': paths})

class Transaction(StoreInterface):
    def __init__(self, base=None):
        self._base = base

    def commit(self):
        pass

class PensiveClient(object):
    pass
