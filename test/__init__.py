'''
Global colored logging configuration for testing.
'''
import logging, traceback, sys
from rainbow_logging_handler import RainbowLoggingHandler

handler = RainbowLoggingHandler(sys.stderr)
handler.setFormatter(logging.Formatter('%(asctime)s\t[%(name)s] %(pathname)s:%(lineno)d\t%(levelname)s:\t%(message)s'))

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)

logging.getLogger('src.pensive.core').setLevel(logging.ERROR)