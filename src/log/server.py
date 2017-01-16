import logging

import socket

import jsonschema

import httplib

import re

from tornado.tcpserver import TCPServer
from tornado.ioloop import IOLoop
from tornado.iostream import StreamClosedError
from tornado.escape import json_decode
from tornado.web import RequestHandler, StaticFileHandler, Application
from tornado import gen

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql.expression import and_, or_

from . import INTEGER_ATTR, FLOAT_ATTR, STRING_ATTR, TEXT_ATTR, ALL_ATTR, DEFAULT_RECORD_PORT, DEFAULT_WEB_PORT

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

Base = declarative_base()

def _make_json_schema():
    schema = {
        '$schema': 'http://json-schema.org/draft-04/schema#',
        'type': 'object',
        'properties': {
        },
        'required': ['levelno', 'lineno', 'name', 'pathname', 'filename', 'module', 'funcName', 'message'],
    }

    for attr in INTEGER_ATTR + FLOAT_ATTR:
        schema['properties'][attr] = {'type': 'number'}
    for attr in STRING_ATTR + TEXT_ATTR:
        schema['properties'][attr] = {'type': 'string'}

    return schema

def _make_sqlalchemy_schema():
    from sqlalchemy import Column, Float, Integer
    from .regex import String, Text

    schema = {
        '__tablename__': 'record',
        'id': Column(Integer, primary_key=True, autoincrement=True),
        'ip': Column(String(2014)),
        'hostname': Column(String(2014)),
        'port': Column(Integer)
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

        try:
            hostname = socket.gethostbyaddr(address[0])[0]
        except socket.herror:
            hostname = None

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
            (record.ip, record.port) = address
            record.hostname = hostname or record.ip

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

def _record_to_dict(record):
    obj = {
        'id': record.id,
        'hostname': record.hostname,
        'ip': record.ip,
        'port': record.port,
    }

    for attr in ALL_ATTR:
        obj[attr] = getattr(record, attr)

    return obj

class RecordsIndexHandler(RequestHandler):
    def get(self):
        records = self.application.session.query(LogRecord)

        names = [ r.name for r in records.group_by(LogRecord.name) ]

        level = self.get_query_argument('level', None)
        if level:
            try:
                records = records.filter(LogRecord.levelno >= int(level))
            except ValueError:
                pass

        filters = self.get_query_argument('filter', None)
        if filters:
            for f in filters.split('!'):
                name, _, level = f.partition(':')
                try:
                    records = records.filter(~and_(LogRecord.name == name, LogRecord.levelno < int(level)))
                except ValueError:
                    pass

        search = self.get_query_argument('search', None)
        if search:
            try:
                # validate regex syntax
                re.compile(search)
            except re.error:
                regex = False
            else:
                regex = True

            if regex:
                # apply as regex filter
                records = records.filter(or_(
                    LogRecord.name.iregexp(search),
                    LogRecord.pathname.iregexp(search),
                    LogRecord.hostname.iregexp(search),
                    LogRecord.message.iregexp(search),
                    LogRecord.exception.iregexp(search)
                ))
            else:
                # apply as like filter
                search = search.replace('*', '%').replace('?', '_')
                records = records.filter(or_(
                    LogRecord.name.like(search),
                    LogRecord.pathname.like(search),
                    LogRecord.hostname.like(search),
                    LogRecord.message.like(search),
                    LogRecord.exception.like(search)
                ))

        order = self.get_query_argument('order', 'created')
        if order in ALL_ATTR:
            field = getattr(LogRecord, order)
            records = records.order_by(field)

        available = records.count()

        count = self.get_query_argument('count', None)
        if count:
            try:
                records = records.limit(int(count))
            except ValueError:
                pass

        skip = self.get_query_argument('skip', None)
        if skip:
            try:
                skip = int(skip)
            except ValueError:
                pass
            else:
                if skip < 0:
                    skip = max([0, available + skip])

                records = records.offset(skip)

        objs = [_record_to_dict(r) for r in records]

        self.write({'records': objs, 'available': available, 'skip': skip or 0, 'names': names})

class RecordHandler(RequestHandler):
    def get(self, id):
        record = self.application.session.query(LogRecord).get(id)

        if not record:
            self.send_error(httplib.NOT_FOUND)
        else:
            self.write({'record': _record_to_dict(record)})

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
        self.add_handlers(r'.*', [
            (r'/record/(?P<id>\d*)/*', RecordHandler),
            (r'/records/*', RecordsIndexHandler),
            (r'//*()', StaticFileHandler, {'path': 'src/log/viewer.html'}),
            (r'/js/viewer.js()', StaticFileHandler, {'path': 'src/log/viewer.js'}),
            (r'/(.*)/*', StaticFileHandler, {'path': 'data/log/static/'})
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
