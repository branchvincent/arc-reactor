import logging

import jsonschema

import httplib

from tornado.tcpserver import TCPServer
from tornado.ioloop import IOLoop
from tornado.iostream import StreamClosedError
from tornado.escape import json_decode
from tornado.web import RequestHandler, StaticFileHandler, Application
from tornado import gen

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

from . import INTEGER_ATTR, FLOAT_ATTR, STRING_ATTR, TEXT_ATTR, DEFAULT_RECORD_PORT, DEFAULT_WEB_PORT

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

Base = declarative_base()

def _make_json_schema():
    schema = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
        },
        #'required': ['levelno', 'lineno', 'name', 'pathname', 'filename', 'module', 'funcName', 'message'],
    }

    for attr in INTEGER_ATTR + FLOAT_ATTR:
        schema['properties'][attr] = {'type': 'number'}
    for attr in STRING_ATTR + TEXT_ATTR:
        schema['properties'][attr] = {'type': 'string'}

    return schema

def _make_sqlalchemy_schema():
    from sqlalchemy import Column, Float, Integer, String, Text

    schema = {
        '__tablename__': 'record',
        'id': Column(Integer, primary_key=True, autoincrement=True),
        'host': Column(String(2014)),
    }

    for attr in INTEGER_ATTR:
        schema[attr] = Column(Integer)
    for attr in FLOAT_ATTR:
        schema[attr] = Column(Float)
    for attr in STRING_ATTR:
        schema[attr] = Column(String(1024))
    for attr in TEXT_ATTR:
        schema[attr] = Column(Text)

    return type('LogRecord', (Base,), schema)

LogRecord = _make_sqlalchemy_schema()  # pylint: disable=invalid-name

class LogRecordServer(TCPServer):
    SCHEMA = _make_json_schema()

    def __init__(self, port=None, address=None):
        self._port = port or DEFAULT_RECORD_PORT
        self._address = address or ''

        # open the database
        self.engine = create_engine('sqlite:///db/records.sqlite')
        self.engine.connect()

        # ensure tables are created
        Base.metadata.create_all(self.engine)

        # open session
        self.session = sessionmaker(bind=self.engine)()

        super(LogRecordServer, self).__init__()

    @gen.coroutine
    def handle_stream(self, stream, address):
        logger.info('connection {}:{}'.format(*address))

        while True:
            try:
                line = yield stream.read_until('\n')
            except StreamClosedError:
                break

            try:
                obj = json_decode(line)
                jsonschema.validate(obj, self.SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed record line: {}\n\n'
                               'Line:\n{}'.format(exc, line))
                break

            record = LogRecord()
            record.host = '{}:{}'.format(*address)

            for attr in INTEGER_ATTR + FLOAT_ATTR:
                setattr(record, attr, obj.get(attr, 0))
            for attr in STRING_ATTR + TEXT_ATTR:
                setattr(record, attr, obj.get(attr, ''))

            self.session.add(record)
            self.session.commit()

        logger.info('disconnect {}:{}'.format(*address))
        if not stream.closed():
            stream.close()

    def run(self, loop=None):
        if not loop:
            loop = IOLoop.current()

        loop.make_current()

        # bind the socket
        self.listen(self._port, self._address)
        logger.info('Logger started on {}:{}'.format(
            self._address or '*', self._port))

        try:
            loop.start()
        except KeyboardInterrupt:
            pass

        loop.stop()
        loop.close()

        logger.info('Logger stopped')

class RecordsIndexHandler(RequestHandler):
    def get(self):
        records = self.application.session.query(LogRecord).order_by(LogRecord.created)

        objs = []
        for record in records:
            obj = {'id': record.id, 'host': record.host}

            for attr in INTEGER_ATTR + FLOAT_ATTR + STRING_ATTR + TEXT_ATTR:
                obj[attr] = getattr(record, attr)

            objs.append(obj)

        self.write({'records': objs})

class RecordHandler(RequestHandler):
    def get(self, id):
        records = self.application.session.query(LogRecord).filter_by(id=id)

        if not records:
            self.send_error(httplib.NOT_FOUND)
        else:
            record = records.first()

            obj = {'id': id, 'host': record.host}
            for attr in INTEGER_ATTR + FLOAT_ATTR + STRING_ATTR + TEXT_ATTR:
                obj[attr] = getattr(record, attr)

            self.write({'record': obj})

class LogWebServer(Application):
    def __init__(self, port=None, address=None):
        super(LogWebServer, self).__init__()
        self._port = port or DEFAULT_WEB_PORT
        self._address = address

        # open the database
        self.engine = create_engine('sqlite:///db/records.sqlite')
        self.engine.connect()

        # ensure tables are created
        Base.metadata.create_all(self.engine)

        # open session
        self.session = sessionmaker(bind=self.engine)()

        # install handlers for various URLs
        self.add_handlers(r'.*', [(r'/record/(?P<id>\d*)/*', RecordHandler)])
        self.add_handlers(r'.*', [(r'/records/*', RecordsIndexHandler)])
        self.add_handlers(r'.*', [(r'/(.*)/*', StaticFileHandler, {'path': 'src/log'})])

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
        if not loop:
            loop = IOLoop.current()

        loop.make_current()

        # bind the socket
        self.listen(self._port, self._address)
        logger.info('Logger started on {}:{}'.format(
            self._address or '*', self._port))

        try:
            loop.start()
        except KeyboardInterrupt:
            pass

        loop.stop()
        loop.close()

        logger.info('Logger stopped')
