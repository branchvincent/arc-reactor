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

def compute(full_cloud, object_clouds, store=None, clean=True):
    # make a workspace
    tmp_path = mkdtemp(prefix='vacuum_')
    segment_path = os.path.join(tmp_path, 'segments')
    target_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'evaluate.py')

    logger.debug('working in {}'.format(tmp_path))

    # write each object point cloud to disk in PCD format
    os.mkdir(segment_path)
    for (i, cloud) in enumerate(object_clouds):
        pcd.write(numpy.array(cloud), os.path.join(segment_path, '{}.pcd'.format(i)))

    # write the full point cloud to disk in NumPy format
    numpy.save(os.path.join(tmp_path, 'pc.npy'), full_cloud)

    # invoke evaluation
    check_call([sys.executable, target_path], cwd=tmp_path)

    # load the results
    grasps = []
    for result in json.load(open(os.path.join(tmp_path, 'planes.txt'))):
        grasps.append((
            result['score'],
            result['center'],
            result['orientation'][:3]
        ))
    grasps.sort(key=lambda x: -x[0])

    if store:
        store.put('/debug/grasps', grasps)

    if clean:
        # clean up
        rmtree(tmp_path)

    return grasps