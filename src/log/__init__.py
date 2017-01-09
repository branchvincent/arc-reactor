INTEGER_ATTR = ['levelno', 'lineno', 'thread', 'process']
FLOAT_ATTR = ['created', 'msecs', 'relativeCreated']
STRING_ATTR = ['name', 'pathname', 'filename', 'module', 'funcName', 'threadName', 'processName']
TEXT_ATTR = ['message']

ALL_ATTR = INTEGER_ATTR + FLOAT_ATTR + STRING_ATTR + TEXT_ATTR

DEFAULT_RECORD_PORT = 7777
DEFAULT_WEB_PORT = 9999
