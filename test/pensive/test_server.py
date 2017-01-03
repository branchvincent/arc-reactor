# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

import httplib

from tornado.escape import json_decode, json_encode
from tornado.testing import AsyncHTTPTestCase

from pensive.core import Store
from pensive.server import PensiveServer

class ServerTest_Store_GET(AsyncHTTPTestCase):

    def get_app(self):
        server = PensiveServer()
        server.stores[None].put('', {'a': 4, 'b': {'c': 2}})
        return server

    def test_server_store_get_single(self):
        response = self.fetch('/d/')
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'value': {'a': 4, 'b': {'c': 2}}})

    def test_server_store_get_nested(self):
        response = self.fetch('/d/b/c')
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'value': 2})

    def test_server_store_get_invalid(self):
        response = self.fetch('/i/random/')
        self.assertEqual(response.code, httplib.NOT_FOUND)

class ServerTest_Store_POST(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        self.server.stores[None].put('', {'a': 4, 'b': {'c': 2}})
        return self.server

    def test_server_store_get(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': ['a', 'b/c']}))
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'a': 4, 'b/c': 2})

    def test_server_store_get_relative(self):
        response = self.fetch('/d/b', method='POST', body=json_encode({'operation': 'get', 'keys': ['c']}))
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'c': 2})

    def test_server_store_get_relative2(self):
        response = self.fetch('/d/b/', method='POST', body=json_encode({'operation': 'get', 'keys': ['c']}))
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'c': 2})

    def test_server_store_get_relative3(self):
        response = self.fetch('/d/b/', method='POST', body=json_encode({'operation': 'get', 'keys': ['/c/']}))
        self.assertEqual(response.code, httplib.OK)
        self.assertDictEqual(json_decode(response.body), {'/c/': 2})

    def test_server_store_get_invalid(self):
        response = self.fetch('/d/', method='POST', body='')
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

    def test_server_store_delete_multi(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'delete', 'keys': ['a', 'b/c']}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertIsNone(self.server.stores[None].get())

    def test_server_store_post_schema(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': []}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': [1]}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'asdfad': ['a']}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='POST', body=json_encode({'random': 'get', 'keys': ['a']}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'random', 'keys': ['a']}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'invalid operation')

    def test_server_store_post_invalid(self):
        response = self.fetch('/i/random/', method='POST', body=json_encode({'operation': 'get', 'keys': ['a', 'b/c']}))
        self.assertEqual(response.code, httplib.NOT_FOUND)
        self.assertEqual(response.reason, 'unrecognized instance')

class ServerTest_Store_PUT(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        return self.server

    def test_server_store_put_single(self):
        response = self.fetch('/d/key', method='PUT', body=json_encode({'value': 1234}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertDictEqual(self.server.stores[None].get(), {'key': 1234})

    def test_server_store_put_multi(self):
        response = self.fetch('/d/', method='PUT', body=json_encode({'keys': {'a': 1234, 'b/c': 5678}}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertDictEqual(self.server.stores[None].get(), {'a': 1234, 'b': {'c': 5678}})

    def test_server_store_put_multi_relative(self):
        response = self.fetch('/d/rel', method='PUT', body=json_encode({'keys': {'a': 1234, 'b/c': 5678}}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertDictEqual(self.server.stores[None].get(), {'rel': {'a': 1234, 'b': {'c': 5678}}})

    def test_server_store_put_multi_relative2(self):
        response = self.fetch('/d/rel/', method='PUT', body=json_encode({'keys': {'a/': 1234, '/b/c/': 5678}}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertDictEqual(self.server.stores[None].get(), {'rel': {'a': 1234, 'b': {'c': 5678}}})

    def test_server_store_put_schema(self):
        response = self.fetch('/d/', method='PUT', body=json_encode({'keys': []}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='PUT', body=json_encode({'keys': 1234}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/d/', method='PUT', body=json_encode({'random': {'a/': 1234, '/b/c/': 5678}}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'incomplete payload')

    def test_server_store_put_invalid(self):
        response = self.fetch('/i/random/', method='PUT', body=json_encode({'keys': {'a/': 1234, '/b/c/': 5678}}))
        self.assertEqual(response.code, httplib.NOT_FOUND)
        self.assertEqual(response.reason, 'unrecognized instance')

class ServerTest_Store_DELETE(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        self.server.stores[None].put('', {'key': 1234, 'key2': 5678})
        return self.server

    def test_server_store_delete_single(self):
        response = self.fetch('/d/key', method='DELETE')
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertDictEqual(self.server.stores[None].get(), {'key2': 5678})

    def test_server_store_put_invalid(self):
        response = self.fetch('/i/random/', method='DELETE')
        self.assertEqual(response.code, httplib.NOT_FOUND)
        self.assertEqual(response.reason, 'unrecognized instance')

class ServerTest_Manager_GET(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        self.server.stores['a'] = Store({'list': [1, 2]})
        self.server.stores['b'] = Store()
        return self.server

    def test_server_manager_index(self):
        response = self.fetch('/s/')
        self.assertEqual(response.code, httplib.OK)
        self.assertItemsEqual(json_decode(response.body)['index'], [None, 'a', 'b'])

    def test_server_manager_index_invalid(self):
        response = self.fetch('/s/random')
        self.assertEqual(response.code, httplib.NOT_FOUND)

    def test_server_manager_create_empty(self):
        response = self.fetch('/s/c', method='PUT', body='')
        self.assertEqual(response.code, httplib.CREATED)
        self.assertItemsEqual(self.server.stores.keys(), [None, 'a', 'b', 'c'])
        store = self.server.stores['c']

        response = self.fetch('/s/c', method='PUT', body='')
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertItemsEqual(self.server.stores.keys(), [None, 'a', 'b', 'c'])
        self.assertIs(store, self.server.stores['c'])

    def test_server_manager_create_default(self):
        store = self.server.stores[None]
        response = self.fetch('/s/', method='PUT', body='')
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertItemsEqual(self.server.stores.keys(), [None, 'a', 'b'])
        self.assertIs(store, self.server.stores[None])

    def test_server_manager_create_fork(self):
        value = self.server.stores['a'].get('list')
        response = self.fetch('/s/c', method='PUT', body=json_encode({'parent': 'a'}))
        self.assertEqual(response.code, httplib.CREATED)
        self.assertItemsEqual(self.server.stores.keys(), [None, 'a', 'b', 'c'])
        self.assertIs(value, self.server.stores['c'].get('list'))

        value_copy = value[:]
        response = self.fetch('/i/c/list', method='PUT', body=json_encode({'value': 1}))
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertItemsEqual(value_copy, value)

    def test_server_manager_create_fork_schema(self):
        response = self.fetch('/s/c', method='PUT', body=json_encode({'random': 'a'}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

        response = self.fetch('/s/c', method='PUT', body=json_encode({'parent': 1}))
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'malformed payload')

    def test_server_manager_create_fork_nonexisting(self):
        response = self.fetch('/s/c', method='PUT', body=json_encode({'parent': 'nonexisting'}))
        self.assertEqual(response.code, httplib.NOT_FOUND)
        self.assertEqual(response.reason, 'unrecognized instance')

    def test_server_manager_delete(self):
        response = self.fetch('/s/a', method='DELETE')
        self.assertEqual(response.code, httplib.NO_CONTENT)
        self.assertItemsEqual(self.server.stores.keys(), [None, 'b'])

    def test_server_manager_delete_default(self):
        response = self.fetch('/s', method='DELETE')
        self.assertEqual(response.code, httplib.BAD_REQUEST)
        self.assertEqual(response.reason, 'cannot delete default store')

    def test_server_manager_delete_nonexisting(self):
        response = self.fetch('/s/random', method='DELETE')
        self.assertEqual(response.code, httplib.NOT_FOUND)
        self.assertEqual(response.reason, 'unrecognized instance')
