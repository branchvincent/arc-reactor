'''
Web server interface for Pensive.

The server uses the following URL structure.



'''

import logging
logger = logging.getLogger(__name__)

import json

from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler

from .core import Store

class StoreManager(object):

    def __init__(self):
        self._store = Store

class PensiveHandler(BaseHTTPRequestHandler):

    protocol_version = 'HTTP/1.1'
    server_version = 'Pensive/1.0'

    def do_GET(self):
        self.send_json(self.path)

    def send_json(self, obj, code=200, headers=None):
        if not headers:
            headers = {}
        # indicate JSON content type
        headers['Content-Type'] = 'application/json; charset=utf-8'
        # generate the response
        self.send_response(json.dumps(
            obj, indent=4, sort_keys=True, encoding='utf-8'), code, headers)

    def send_response(self, data, code=200, headers=None):
        if not headers:
            headers = {}

        # set standard headers if not overridden
        headers.setdefault('Server', self.version_string())
        headers.setdefault('Date', self.date_time_string())

        if data:
            # default to plaintext response
            headers.setdefault('Content-Type', 'text/plain; charset=utf-8')
            # generate content length if not set
            headers.setdefault('Content-Length', len(data))

        # log the message early in case the response goes badly
        self.log_request(code, headers.get('Content-Length', '-'))

        response = []

        # send the status line
        message = self.responses.get(code, ('',))[0]
        response.append('{} {} {}'.format(
            self.protocol_version, code, message))

        # send standard headers first
        for k in ['Server', 'Date', 'Content-Type', 'Content-Length']:
            if k in headers:
                response.append('{}: {}'.format(k, headers.pop(k)))
        # send the remaining headers
        for k in sorted(headers.keys()):
            response.append('{}: {}'.format(k, headers.pop(k)))

        # add delimiting newline
        response.append('')

        # add payload if given
        if data:
            response.append(data)

        # send the head and payload
        self.wfile.write('\r\n'.join(response))

    def log_message(self, format, *args):
        logger.info(format % args)

    def log_error(self, format, *args):
        logger.error(format % args)

class PensiveServer(HTTPServer):
    def __init__(self, server_address):
        HTTPServer.__init__(self, server_address, PensiveHandler)

        self.manager = StoreManager()
