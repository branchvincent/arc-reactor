from tornado.escape import json_decode, json_encode
from tornado.testing import AsyncHTTPTestCase

from pensive.server import PensiveServer

class ServerTest_GET(AsyncHTTPTestCase):

    def get_app(self):
        server = PensiveServer()
        server.manager.stores[None].put('', {'a': 4, 'b': {'c': 2}})
        return server

    def test_server_get_single(self):
        response = self.fetch('/d/')
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'value': {'a': 4, 'b': {'c': 2}}})

    def test_server_get_nested(self):
        response = self.fetch('/d/b/c')
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'value': 2})
class ServerTest_POST(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        self.server.manager.stores[None].put('', {'a': 4, 'b': {'c': 2}})
        return self.server

    def test_server_get(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': ['a', 'b/c']}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'a': 4, 'b/c': 2})

    def test_server_get_relative(self):
        response = self.fetch('/d/b', method='POST', body=json_encode({'operation': 'get', 'keys': ['c']}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'c': 2})

    def test_server_get_relative2(self):
        response = self.fetch('/d/b/', method='POST', body=json_encode({'operation': 'get', 'keys': ['c']}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'c': 2})

    def test_server_get_invalid(self):
        response = self.fetch('/d/', method='POST', body='')
        self.assertEqual(response.code, 400)

    def test_server_delete_multi(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'delete', 'keys': ['a', 'b/c']}))
        self.assertEqual(response.code, 200)
        self.assertIsNone(self.server.manager.stores[None].get())

    def test_server_post_schema(self):
        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': []}))
        self.assertEqual(response.code, 400)

        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'keys': [1]}))
        self.assertEqual(response.code, 400)

        response = self.fetch('/d/', method='POST', body=json_encode({'operation': 'get', 'asdfad': ['a']}))
        self.assertEqual(response.code, 400)

        response = self.fetch('/d/', method='POST', body=json_encode({'random': 'get', 'keys': ['a']}))
        self.assertEqual(response.code, 400)

class ServerTest_PUT(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        return self.server

    def test_server_put_single(self):
        response = self.fetch('/d/key', method='PUT', body=json_encode({'value': 1234}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(self.server.manager.stores[None].get(), {'key': 1234})

    def test_server_put_multi(self):
        response = self.fetch('/d/', method='PUT', body=json_encode({'keys': {'a': 1234, 'b/c': 5678}}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(self.server.manager.stores[None].get(), {'a': 1234, 'b': {'c': 5678}})

    def test_server_put_multi_relative(self):
        response = self.fetch('/d/rel', method='PUT', body=json_encode({'keys': {'a': 1234, 'b/c': 5678}}))
        self.assertEqual(response.code, 200)
        self.assertDictEqual(self.server.manager.stores[None].get(), {'rel': {'a': 1234, 'b': {'c': 5678}}})

class ServerTest_DELETE(AsyncHTTPTestCase):

    def get_app(self):
        self.server = PensiveServer()
        self.server.manager.stores[None].put('', {'key': 1234, 'key2': 5678})
        return self.server

    def test_server_delete_single(self):
        response = self.fetch('/d/key', method='DELETE')
        self.assertEqual(response.code, 200)
        self.assertDictEqual(self.server.manager.stores[None].get(), {'key2': 5678})
