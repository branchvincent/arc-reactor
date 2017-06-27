import logging

from subprocess import check_call, CalledProcessError
import os

from time import sleep

logger = logging.getLogger(__name__)

class CameraAcquisitionError(Exception):
    pass

def _build_command(name):
    return ['python3', 'reactor', 'shell', 'perception.perception', '-f', name]

def _build_args(prefix, args):
    return [prefix] + [str(a) for a in args]

def initialize_cameras(serials):
    args = _build_command('update_cams')
    args += _build_args('-sn', serials)

    logger.debug('invoking camera initialization: {}'.format(args))
    check_call(args)

def acquire_images(store, serials, photo_urls):
    args = _build_command('acquire_images')
    args += _build_args('-sn', serials)
    args += _build_args('-u', photo_urls)

    logger.debug('invoking camera acquisition: {}'.format(args))

    try:
        check_call(args)
    except CalledProcessError:
        logger.exception("Subprocess died")
        raise CameraAcquisitionError()

def acquire_images_new(store, serials, photo_urls):
    # wait for prior acquisition to end
    while store.get('/acquire_images/run', False):
        logger.warn('waiting for acquire images idle')
        sleep(0.5)

    # update parameters
    store.put('/acquire_images/serial_numbers', serials)
    store.put('/acquire_images/urls', photo_urls)

    # trigger recognition
    store.put('/acquire_images/done', False)
    store.put('/acquire_images/run', True)

    logger.debug('acquire images started')

    # wait for completion
    while not store.get('/acquire_images/done', False):
        sleep(0.1)

    error = store.get('/acquire_images/error')
    if error:
        raise RuntimeError('image acquisition failed: {}'.format(error))

    logger.debug('acquire images finished')

def segment_images(photo_urls, bounds_urls, bounds_pose_urls):
    args = _build_command('segment_images')
    args += _build_args('-u', photo_urls)
    args += _build_args('-b', bounds_urls)
    args += _build_args('-x', bounds_pose_urls)

    logger.debug('invoking image segmentation: {}'.format(args))
    check_call(args)

def recognize_objects(store, photo_urls, locations):
    # wait for prior recognition to end
    while store.put('/object_recognition/run', False):
        logger.warn('waiting for object recognition idle')
        sleep(0.5)

    # update parameters
    store.put('/object_recognition/urls', photo_urls)
    store.put('/object_recognition/locations', locations)

    # trigger recognition
    store.put('/object_recognition/done', False)
    store.put('/object_recognition/run', True)

    logger.debug('objection recognition started')

    # wait for completion
    while not store.get('/object_recognition/done', False):
        sleep(0.1)

    logger.debug('object recognition finished')
