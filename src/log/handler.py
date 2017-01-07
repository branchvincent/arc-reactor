import logging.handlers

from tornado.escape import json_encode

from . import INTEGER_ATTR, FLOAT_ATTR, STRING_ATTR, TEXT_ATTR, DEFAULT_RECORD_PORT

class LogServerHandler(logging.handlers.SocketHandler):
    def filter(self, record):
        return not record.name.startswith('log')

    def makePickle(self, record):
        obj = {}
        for attr in INTEGER_ATTR + FLOAT_ATTR + STRING_ATTR + TEXT_ATTR:
            if attr == 'message':
                obj[attr] = record.getMessage()
            else:
                obj[attr] = getattr(record, attr)

        # include the trailing newline to indicate the end of a record
        return json_encode(obj) + '\n'
