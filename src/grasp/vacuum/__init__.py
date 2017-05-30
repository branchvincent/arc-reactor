import os
import sys
from shutil import rmtree
from tempfile import mkdtemp

from subprocess import check_call
import runpy
import json

import logging

import numpy

from util import pcd

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

def compute(full_cloud, object_masks, store=None, clean=True, aligned_color=None):
    # make a workspace
    tmp_path = mkdtemp(prefix='vacuum_')
    segment_path = os.path.join(tmp_path, 'segments')
    target_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'evaluate.py')

    logger.debug('working in {}'.format(tmp_path))

    # write each object point cloud to disk in PCD format
    os.mkdir(segment_path)
    for (i, mask) in enumerate(object_masks):
        numpy.save(os.path.join(segment_path, '{}.npy'.format(i)), mask)

    # write the full point cloud to disk in NumPy format
    numpy.save(os.path.join(tmp_path, 'pc.npy'), full_cloud)

    if aligned_color is not None:
        import cv2
        cv2.imwrite(os.path.join(tmp_path, 'bin.png'), aligned_color[..., ::-1])

    # invoke evaluation
    check_call([sys.executable, target_path], cwd=tmp_path)

    # load the results
    grasps = json.load(open(os.path.join(tmp_path, 'planes.txt')))
    #grasps.sort(key=lambda x: -x[0])

    if store:
        store.put('/debug/grasps', grasps)

    if clean:
        # clean up
        rmtree(tmp_path)

    return grasps
