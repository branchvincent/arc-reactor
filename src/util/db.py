from gzip import GzipFile

from time import time

import logging

from pensive.client import json_encode

logger = logging.getLogger(__name__)

def dump(store, path, compress=True, timestamp=True):
    if timestamp:
        path += '-{:.0f}'.format(1000 * time())

    if not path.endswith('.json') or path.endswith('.json.gz'):
        path += '.json'

    if compress:
        if not path.endswith('.gz'):
            path += '.gz'

    logger.info('dumping database to "{}"'.format(path))

    obj = store.get()
    GzipFile('', fileobj=open(path, 'wb')).write(json_encode(obj))
