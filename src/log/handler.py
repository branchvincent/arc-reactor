'''
Python logging handler to send records to the Log Server.
'''

import logging.handlers

import traceback

from tornado.escape import json_encode

from . import ALL_ATTR

class LogServerHandler(logging.handlers.SocketHandler):


    def filter(self, record):
        '''
        Override to prevent the log server from capturing its own messages.`
        '''
        return not record.name.startswith('log')

    def makePickle(self, record):
        '''
        Format the record in JSON instead of Pickle for the log server.
        '''

        print record.levelno

        obj = {}
        # create a dictionary of all record details
        for attr in ALL_ATTR:
            if attr == 'message':
                # get the formatted record message
                obj[attr] = record.getMessage().strip()

            elif attr == 'exception':
                # format exception info if provided
                if record.exc_info:
                    obj[attr] = '\n'.join(traceback.format_exception(*record.exc_info[:3]))

            else:
                # normal attribute
                obj[attr] = getattr(record, attr)

        # treat multi-lined messages as exception details if no exception detail provided
        if 'exception' not in obj and '\n' in obj['message']:
            obj['message'], _, obj['exception'] = obj['message'].partition('\n')
            obj['exception'] = obj['exception'].strip()

        # include the trailing newline to indicate the end of a record
        return json_encode(obj) + '\n'
