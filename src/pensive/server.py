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

import httplib

from tornado.ioloop import IOLoop
from tornado.web import RequestHandler, StaticFileHandler, Application
from tornado.escape import json_decode

import jsonschema

from .core import Store
from . import DEFAULT_PORT

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ManagerHandler(RequestHandler):  # pylint: disable=abstract-method
    '''
    Handler for GET, PUT, and DELETE for manipulating `Store` instances.
    '''

    PUT_SCHEMA = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
            'parent': {
                'type': ['string', 'null'],
            },
        },
        'required': ['parent'],
    }

    def get(self, instance=None):  # pylint: disable=arguments-differ
        '''
        Access the store index.
        '''

        if instance:
            # no utility for providing instance
            self.send_error(httplib.NOT_FOUND)
        else:
            # send back the list of keys
            self.write({'index': self.application.stores.keys()})

    def put(self, instance=None):  # pylint: disable=arguments-differ
        '''
        Create a new store named `instance`.

        If a request body is provided, it is decoded as a JSON object
        to specify the parent store. The JSON object must has the structure
        below.
        ```
        {
            'parent': <instance>
        }
        ```
        If `parent is None`, the default instance is used.

        If no request body is provided, an empty store is created.

        If the `instance` already exists, no action is taken.
        '''

        if instance in self.application.stores:
            # no action if instance already exists
            self.set_status(httplib.NO_CONTENT)
            logger.warning('silently ignored store re-creation request: "{}"'.format(instance))
        elif self.request.body:
            try:
                # decode and validate the request JSON
                obj = json_decode(self.request.body)
                jsonschema.validate(obj, self.PUT_SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed payload: {}\n\nPayload:\n{}'.format(exc, self.request.body))
                self.send_error(400, reason='malformed payload')
            else:
                try:
                    # find the parent
                    parent = self.application.stores[obj['parent']]
                except KeyError:
                    logger.warning('unrecognized instance: {}'.format(instance))
                    self.send_error(404, reason='unrecognized instance')
                else:
                    # fork the new instance
                    self.application.stores[instance] = parent.fork()
                    self.set_status(httplib.CREATED)
                    logger.info('forked a new store "{}" -> "{}"'.format(obj['parent'], instance))
        else:
            # create an empty new store
            self.application.stores[instance] = Store()
            self.set_status(httplib.CREATED)
            logger.info('created empty new store "{}"'.format(instance))

    def delete(self, instance=None):  # pylint: disable=arguments-differ
        '''
        Delete the store named `instance`.

        The default store cannot be deleted.
        '''

        if instance is None:
            self.send_error(httplib.BAD_REQUEST, reason='cannot delete default store')
            logger.warning('attempted default store deletion')
        else:
            try:
                # delete the instance
                del self.application.stores[instance]
            except KeyError:
                logger.warning('unrecognized instance: {}'.format(instance))
                self.send_error(httplib.NOT_FOUND, reason='unrecognized instance')
            else:
                logger.info('deleted store "{}"'.format(instance))
                self.set_status(httplib.NO_CONTENT)

class StoreHandler(RequestHandler):  # pylint: disable=abstract-method
    '''
    Handler for GET, POST, PUT, and DELETE for accessing the `Store`.
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
        '''
        Decode JSON query arguments.
        '''

        self.options = {}
        for k in self.request.query_arguments:
            data = self.get_query_argument(k)
            try:
                self.options[k] = json_decode(data)
            except ValueError as exc:
                logger.warning('bad JSON in query argument "{}": {}\n\nArgument:\n{}'.format(k, exc, data))
                self.send_error(httplib.BAD_REQUEST, reason='malformed query string')

    def get(self, path, instance=None):  # pylint: disable=arguments-differ
        '''
        Handle GET requests.

        If `instance is None`, the default store is accessed.

        All query arguments are interpreted as JSON-encoded options.

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
            self.send_error(httplib.NOT_FOUND, reason='unrecognized instance')
        else:
            try:
                # retrieve value
                self.write({'value': store.get(path, **self.options)})
            except KeyError:
                self.send_error(httplib.NOT_FOUND)
            except TypeError as exc:
                logger.warning('unrecognized option: {}\n\nOptions:\n{}'.format(exc, self.options))
                self.send_error(httplib.BAD_REQUEST, reason='unrecognized option')

    def post(self, path, instance=None):  # pylint: disable=arguments-differ
        '''
        Handle POST requests for multiple GET and DELETE.

        If `instance is None`, the default store is accessed.

        The request body is decoded as JSON to manipulate multiple keys
        concurrently. The JSON must have the structure below.
        ```
        {
            'operation': <command>,
            'keys': [ <key1>, <key2>, ... ],
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
        If the operation is `INDEX`, the response body will be a JSON object
        with the structure below.
        ```
        {
            <key1>: <index1>,
            <key2>: <index2>,
            ...
        }
        ```
        In both cases, the keys are relative to the requested path.

        If the operation is `DELETE`, there is no response body.
        '''

        try:
            # find the store
            store = self.application.stores[instance]
        except KeyError:
            logger.warning('unrecognized instance: {}'.format(instance))
            self.send_error(httplib.NOT_FOUND, reason='unrecognized instance')
        else:
            try:
                # decode and validate the request JSON
                obj = json_decode(self.request.body)
                jsonschema.validate(obj, StoreHandler.POST_SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed payload: {}\n\nPayload:\n{}'.format(exc, self.request.body))
                self.send_error(httplib.BAD_REQUEST, reason='malformed payload')
            else:
                if path and not path.endswith(Store.SEPARATOR):
                    path += Store.SEPARATOR

                if obj['operation'].lower() == 'get':
                    # multiple GET
                    self.write({key: store.get(path + key.strip('/')) for key in obj['keys']})
                elif obj['operation'].lower() == 'index':
                    # multiple INDEX
                    # FIXME: this will cause an error if options['depth'] is not an integer
                    self.write({key: store.index(path + key.strip('/'), depth=self.options.get('depth', -1)) for key in obj['keys']})
                elif obj['operation'].lower() == 'delete':
                    # multiple DELETE
                    for key in obj['keys']:
                        store.delete(path + key.strip('/'))
                    self.set_status(httplib.NO_CONTENT)
                else:
                    logger.warning('invalid operation: {}'.format(obj['operation']))
                    self.send_error(httplib.BAD_REQUEST, reason='invalid operation')

    def put(self, path, instance=None):  # pylint: disable=arguments-differ
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
            self.send_error(httplib.NOT_FOUND, reason='unrecognized instance')
        else:
            try:
                # decode and validate the request JSON
                obj = json_decode(self.request.body)
                jsonschema.validate(obj, StoreHandler.PUT_SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed payload: {}\n\nPayload:\n{}'.format(exc, self.request.body))
                self.send_error(httplib.BAD_REQUEST, reason='malformed payload')
            else:
                if 'value' in obj:
                    # single put
                    try:
                        store.put(path, obj['value'], **self.options)
                    except KeyError:
                        self.send_error(httplib.NOT_FOUND)
                    except TypeError as exc:
                        logger.warning('unrecognized option: {}\n\nOptions:\n{}'.format(exc, self.options))
                        self.send_error(httplib.BAD_REQUEST, reason='unrecognized option')
                    else:
                        self.set_status(httplib.NO_CONTENT)
                elif 'keys' in obj:
                    if path and not path.endswith(Store.SEPARATOR):
                        path += Store.SEPARATOR
                    # multiple put with relative path
                    for (key, value) in obj['keys'].iteritems():
                        store.put(path + key.strip('/'), value)
                    self.set_status(httplib.NO_CONTENT)
                else:
                    logger.warning('incomplete payload')
                    self.send_error(httplib.BAD_REQUEST, reason='incomplete payload')

    def delete(self, path, instance=None):  # pylint: disable=arguments-differ
        '''
        Handle DELETE requests.

        If `instance is None`, the default store is accessed.
        '''
        try:
            store = self.application.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(httplib.NOT_FOUND, reason='unrecognized instance')
        else:
            # single delete
            try:
                store.delete(path, **self.options)
            except KeyError:
                self.send_error(httplib.NOT_FOUND)
            except TypeError as exc:
                logger.warning('unrecognized option: {}\n\nOptions:\n{}'.format(exc, self.options))
                self.send_error(httplib.BAD_REQUEST, reason='unrecognized option')
            else:
                self.set_status(httplib.NO_CONTENT)

class PensiveServer(Application):
    '''
    Tornado web application for database access over HTTP.
    '''
    def __init__(self, port=None, address=None):
        super(PensiveServer, self).__init__()
        self._port = port or DEFAULT_PORT
        self._address = address

        # create default store
        self.stores = {
            None: Store()
        }

        # install handlers for various URLs
        self.add_handlers(r'.*', [
            (r'/d/(?P<path>.*)/*', StoreHandler),
            (r'/i/(?P<instance>.+?)/(?P<path>.*)/*', StoreHandler),
            (r'/s/*', ManagerHandler),
            (r'/s/(?P<instance>.+?)?/*', ManagerHandler),
            (r'//*()', StaticFileHandler, {'path': 'src/pensive/viewer.html'}),
            (r'/js/viewer.js()', StaticFileHandler, {'path': 'src/pensive/viewer.js'}),
            (r'/(.*)/*', StaticFileHandler, {'path': 'data/web/static/'})
        ])

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
            1000 * handler.request.request_time()))

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
