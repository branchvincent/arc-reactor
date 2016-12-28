from tornado.escape import json_decode, json_encode
from tornado.testing import AsyncHTTPTestCase

from pensive.server import PensiveServer

class ServerTest(AsyncHTTPTestCase):

    port = 8888

    def get_app(self):
        return PensiveServer(self.port)

    def test_server_get(self):
        response = self.fetch('/d/')
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'data': None})

    def test_server_post(self):
        response = self.fetch('/d/key', method='POST', body=json_encode({'data': 1234}))
        self.assertEqual(response.code, 200)

        response = self.fetch('/d/')
        self.assertEqual(response.code, 200)
        self.assertDictEqual(json_decode(response.body), {'data': {'key': 1234}})

    def test_server_delete(self):
        response = self.fetch('/d/key', method='POST', body=json_encode({'data': 1234}))
        self.assertEqual(response.code, 200)

        response = self.fetch('/d/key', method='DELETE')
        self.assertEqual(response.code, 200)
