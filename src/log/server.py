import logging

import socket

import jsonschema

import httplib

import re

import os
import os.path

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
    '''
    Tornado TCP application for receiving `LogRecords` sent by the
    corresponding logging handler.
    '''

    SCHEMA = _make_json_schema()

    def __init__(self, db_url):
        # open the database
        self.engine = create_engine(db_url)
        self.engine.connect()

        # ensure tables are created
        Base.metadata.create_all(self.engine)

        self.sessionmaker = sessionmaker(bind=self.engine)

        super(LogRecordServer, self).__init__()

    @gen.coroutine
    def handle_stream(self, stream, address):
        logger.info('connection {}:{}'.format(*address))

        # perform a one-time hostname resolution
        try:
            hostname = socket.gethostbyaddr(address[0])[0]
        except socket.herror:
            hostname = None

        while True:
            # each record comes as a single line
            try:
                line = yield stream.read_until('\n')
            except StreamClosedError:
                break

            session = self.sessionmaker()

            # decode and valid record
            try:
                obj = json_decode(line)
                jsonschema.validate(obj, self.SCHEMA)
            except (ValueError, jsonschema.ValidationError) as exc:
                logger.warning('malformed record line: {}\n\n'
                               'Line:\n{}'.format(exc, line))
                break

            # build record
            record = LogRecord()
            (record.ip, record.port) = address
            record.hostname = hostname or record.ip

            for attr in INTEGER_ATTR + FLOAT_ATTR:
                setattr(record, attr, obj.get(attr, 0))
            for attr in STRING_ATTR + TEXT_ATTR:
                setattr(record, attr, obj.get(attr, ''))

            # apply
            session.add(record)
            session.commit()

        logger.info('disconnect {}:{}'.format(*address))
        if not stream.closed():
            stream.close()

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
    '''
    `RequestHandler` for serving `LogRecord`s over HTTP with JSON.

    Many filtering and indexing options are available.
     - `level`: the lowest integer `levelno` to return
     - `filters`: a `!`-delimited list of `:`-tuples of logger `name` and lowest `levelno` to return
     - `search`: a regular expression (default) SQL `LIKE` string for searching
     - `order`: record field for sorting
     - `count`: maximum number of records to return
     - `skip`: number of records to skip from start (positive) or end (negative)

     If the `search` parameter compiles as a regular expression, a case-insensitive
     regular expression search on the fields `name`, `pathname`, `hostname`, `message`,
     and `exception` is performed. Otherwise, an SQL `LIKE` operation is performed
     on those fields.

     Returns a dictionary with the matching records, the total number of records
     available, the number of records skipped, and a list of all known logger names.
    '''

    def get(self):
        # create a new session for this request
        session = self.application.sessionmaker()

        records = session.query(LogRecord)

        names = [r.name for r in records.group_by(LogRecord.name)]

        # apply global level filter
        level = self.get_query_argument('level', None)
        if level:
            try:
                records = records.filter(LogRecord.levelno >= int(level))
            except ValueError:
                pass

        # apply filters for specific loggers
        filters = self.get_query_argument('filter', None)
        if filters:
            for f in filters.split('!'):
                name, _, level = f.partition(':')
                try:
                    records = records.filter(~and_(LogRecord.name == name, LogRecord.levelno < int(level)))
                except ValueError:
                    pass

        # apply text matching or regular expressions
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

        # apply field ordering
        order = self.get_query_argument('order', 'created')
        if order in ALL_ATTR:
            field = getattr(LogRecord, order)
            records = records.order_by(field)

        # record the total number of available records before
        # any LIMIT or OFFSET commands reduce it
        available = records.count()

        # apply return limit
        count = self.get_query_argument('count', None)
        if count:
            try:
                records = records.limit(int(count))
            except ValueError:
                pass

        # apply record offset
        skip = self.get_query_argument('skip', None)
        if skip:
            try:
                skip = int(skip)
            except ValueError:
                pass
            else:
                if skip < 0:
                    # treat negative skips as skipping backwards from the end
                    skip = max([0, available + skip])

                records = records.offset(skip)

        # convert all the resulting records into objects for JSON
        objs = [_record_to_dict(r) for r in records]

        # need to send the skip back in case it was negative
        self.write({'records': objs, 'available': available, 'skip': skip or 0, 'names': names})

class RecordHandler(RequestHandler):
    '''
    `RequestHandler` for serving a single `LogRecrod` over HTTP with JSON.
    '''

    def get(self, id):
        record = self.application.session.query(LogRecord).get(id)

        if not record:
            self.send_error(httplib.NOT_FOUND)
        else:
            self.write({'record': _record_to_dict(record)})

class LogWebServer(Application):
    '''
    Tornado HTTP application for serving `LogRecords` over HTTP.
    '''

    def __init__(self, db_url):
        super(LogWebServer, self).__init__()

        # open the database
        self.engine = create_engine(db_url)
        self.engine.connect()

        # ensure tables are created
        Base.metadata.create_all(self.engine)

        self.sessionmaker = sessionmaker(bind=self.engine)

        # install handlers for various URLs
        self.add_handlers(r'.*', [
            (r'/record/(?P<id>\d*)/*', RecordHandler),
            (r'/records/*', RecordsIndexHandler),
            (r'//*()', StaticFileHandler, {'path': 'src/log/viewer.html'}),
            (r'/js/viewer.js()', StaticFileHandler, {'path': 'src/log/viewer.js'}),
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

class LogServer(object):
    def __init__(self, db_url, address=None, web_port=None, record_port=None):
        self._db_url = db_url

        self._address = address
        self._web_port = web_port or DEFAULT_WEB_PORT
        self._record_port = record_port or DEFAULT_RECORD_PORT

    def run(self, loop=None):
        # make the database directory if needed
        if self._db_url.startswith('sqlite:///'):
            path = self._db_url[len('sqlite:///'):]
            path = os.path.dirname(os.path.abspath(path))
            if not os.path.exists(path):
                os.makedirs(path)
                logger.debug('created {}'.format(path))

        if not loop:
            loop = IOLoop.current()

        loop.make_current()

        # create the servers
        LogWebServer(self._db_url).listen(self._web_port, self._address)
        logger.info('Log Web Server started on {}:{}'.format(self._address or '*', self._web_port))

        LogRecordServer(self._db_url).listen(port=self._record_port, address=self._address)
        logger.info('Log Record Server started on {}:{}'.format(self._address or '*', self._record_port))

        try:
            loop.start()
        except KeyboardInterrupt:
            pass

        loop.stop()
        loop.close()

        logger.info('Log Web Server and Log Record Server stopped')
