from unittest import TestCase

from tornado.ioloop import IOLoop

import requests

from threading import Thread

from pensive.server import PensiveServer

def background_server(loop, server):
    loop.make_current()

    server.run()
    loop.start()

class Store_Get(TestCase):

    def setUp(self):
        port = 8888

        self.loop = IOLoop()
        self.thread = Thread(target=background_server, args=(self.loop, PensiveServer(port)))
        self.thread.start()

        self.host = 'http://localhost:{}'.format(port)

    def test_get_root(self):
        r = requests.get(self.host + '/d/')
        self.assertEqual(r.status_code, 200)
        self.assertTrue('application/json' in r.headers['content-type'])
        self.assertEqual(r.encoding.lower(), 'utf-8')
        self.assertDictEqual(r.json(), {'data': None})

    # def test_post_root(self):
    #     r = requests.post(self.host + '/d/key', data={'data': 1234})
    #     self.assertEqual(r.status_code, 200)

    #     r = requests.get(self.host + '/d/')
    #     self.assertEqual(r.status_code, 200)
    #     self.assertDictEqual(r.json(), {'data': {'key': 1234}})

    def tearDown(self):
        self.loop.stop()
