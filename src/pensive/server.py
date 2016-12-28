'''
Web server interface for Pensive.

The server uses the following URL structure.
 * `/d/<path>`: access the default store
 * `/i/<instance>/<path>`: access the named store `instance`
 * `/s/<instance>`: control the named store `instance`

When accessing a store, the HTTP verbs are mapped as follows.
 * `GET`: single `get()`
 * `POST`: multiple `get()` or `delete()`
 * `PUT`: single or multiple `put()`
 * `DELETE`: single `delete()`
Refer to the `StoreHandler` documentation for the expected request
bodies.
'''

import logging

from tornado.ioloop import IOLoop
from tornado.web import RequestHandler, Application, removeslash
from tornado.escape import json_decode

import jsonschema

from .core import Store

logger = logging.getLogger(__name__)

class ManagerHandler(RequestHandler):
    pass

class StoreHandler(RequestHandler):
    '''
    Handler for GET, POST, and DELETE for accessing the `Store`.
    '''

    POST_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
            'operation': {
                'type': 'string'
            },
            'keys': {
                'type': 'array',
                'items': {
                    'type': 'string'
                },
                'minItems': 1,
                'uniqueItems': True
            }
        },
        'required': ['operation', 'keys'],
    }

    PUT_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
            'keys': {
                'type': 'object',
            },
            'value': {}
        },
    }

    def prepare(self):
        pass

    def get(self, path, instance=None):
        '''
        Handle GET requests.

        If `instance is None`, the default store is accessed.

        The response body will be a JSON object with the structure below.
        ```
        {
            'value': <value>
        }
        ```
        '''

        try:
            # find the store
            store = self.application.stores[instance]
        except KeyError:
            logger.warning('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            # retrieve value
            self.write({'value': store.get(path)})

    def post(self, path, instance=None):
        '''
        Handle POST requests for multiple GET and DELETE.

        If `instance is None`, the default store is accessed.

        The request body is decoded as JSON to manipulate multiple keys
        concurrently. The JSON must have the structure below.
        ```
        {
            'operation': <command>
            'keys': [ <key1>, <key2>, ... ]
        }
        ```
        If the operation is `GET`, the response body will be a JSON object
        with the structure below.
        ```
        {
            <key1>: <value1>,
            <key2>: <value2>,
            ...
        }
        ```
        The keys are relative to the requested path.

        If the operation is `DELETE`, there is no response body.
        '''

        try:
            # find the store
            store = self.application.stores[instance]
        except KeyError:
            logger.warning('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            try:
                # decode and validate the request JSON
                obj = json_decode(self.request.body)
                jsonschema.validate(obj, StoreHandler.POST_SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed payload: {}\n\nPayload:\n{}'.format(exc, self.request.body))
                self.send_error(400, reason='malformed payload')
            else:
                if path and not path.endswith(Store.SEPARATOR):
                    path += Store.SEPARATOR

                if obj['operation'].lower() == 'get':
                    # multiple GET
                    self.write({key: store.get(path + key.strip('/')) for key in obj['keys']})
                elif obj['operation'].lower() == 'delete':
                    # multiple DELETE
                    for key in obj['keys']:
                        store.delete(path + key.strip('/'))
                else:
                    logger.warning('invalid operation: {}'.format(obj['operation']))
                    self.send_error(400, reason='invalid operation')

    def put(self, path, instance=None):
        '''
        Handle PUT requests.

        If `instance is None`, the default store is accessed.

        The request body is decoded as JSON to put a single value or to put
        multiple keys concurrently. The JSON must have the structure below.
        ```
        {
            'keys': {
                <key1>: <value1>,
                <key2>: <value2>,
                ...
            }
            -- OR --
            'value': <value>
        }
        ```
        The keys are relative to the requested path.

        If both `keys` and `value` are provided, `keys` is ignored.
        '''
        try:
            store = self.application.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            try:
                # decode and validate the request JSON
                obj = json_decode(self.request.body)
                jsonschema.validate(obj, StoreHandler.PUT_SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed payload: {}\n\nPayload:\n{}'.format(
                    exc, self.request.body))
                self.send_error(400, reason='malformed payload')
            else:
                if 'value' in obj:
                    # single put
                    store.put(path, obj['value'])
                elif 'keys' in obj:
                    if path and not path.endswith(Store.SEPARATOR):
                        path += Store.SEPARATOR
                    # multiple put with relative path
                    for (key, value) in obj['keys'].iteritems():
                        store.put(path + key.strip('/'), value)
                else:
                    logger.warning('incomplete payload')
                    self.send_error(400, reason='incomplete payload')

    def delete(self, path, instance=None):
        '''
        Handle DELETE requests.

        If `instance is None`, the default store is accessed.
        '''
        try:
            store = self.application.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            # single delete
            store.delete(path)


class PensiveServer(Application):
    '''
    Tornado web application for database access over HTTP.
    '''
    def __init__(self, port=8888, address=''):
        super(PensiveServer, self).__init__()
        self._port = port
        self._address = address

        self.stores = {
            None: Store()
        }

        # install handlers for various URLs
        self.add_handlers(r'.*', [(r'/d/(?P<path>.*)/*', StoreHandler)])
        self.add_handlers(r'.*', [(r'/i/(?P<instance>.+)/(?P<path>.*)/*', StoreHandler)])

    def log_request(self, handler):
        '''
        Log the request and response information to module logger.
        '''
        # choose the severity level based on HTTP status codes
        if handler.get_status() < 400:
            log = logger.info
        elif handler.get_status() < 500:
            log = logger.warning
        else:
            log = logger.error

        log('{} {} {} {} {} {:.2f}ms'.format(
            handler.request.remote_ip,
            handler.get_current_user() or '-',
            handler.request.method,
            handler.request.uri,
            handler.get_status(),
            1000 * handler.request.request_time())
        )

    def run(self, loop=None):
        '''
        Start servicing the Tornado event loop.
        '''

        if not loop:
            loop = IOLoop()

        loop.make_current()

        # bind the socket
        self.listen(self._port, self._address)
        logger.info('Pensive started on {}:{}'.format(
            self._address or '*', self._port))

        try:
            loop.start()
        except KeyboardInterrupt:
            pass

        loop.stop()
        loop.close()

        logger.info('Pensive stopped')
