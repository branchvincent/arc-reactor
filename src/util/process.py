import logging

import psutil

logger = logging.getLogger(__name__)

def kill_by_name(name):
    success = True

    for process in psutil.process_iter():
        if name in process.cmdline():
            process.kill()

            try:
                process.wait(timeout=1)
            except psutil.TimeoutExpired:

                logger.error('{} {} resisted killing -> escalating'.format(name, process.pid))
                process.terminate()

                try:
                    process.wait(timeout=1)
                except psutil.TimeoutExpired:
                    success = False
                else:
                    logger.info('killed terminated {} {}'.format(name, process.pid))

            else:
                logger.info('killed {} {}'.format(name, process.pid))

    return success
