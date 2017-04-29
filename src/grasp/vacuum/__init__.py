import os
import sys
from shutil import rmtree

from subprocess import check_call
import runpy
import json

import logging

import numpy

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

BASE_PATH = os.path.abspath(os.path.dirname(__file__))

def _clean():
    # clean up inputs and outputs
    for name in ['pointcloud.pcd', 'pointcloud_downsampled.pcd', 'planes.txt']:
        try:
            os.remove(os.path.join(BASE_PATH, name))
        except OSError:
            pass

    # clean up intermediate files
    rmtree(os.path.join(BASE_PATH, 'segments'), ignore_errors=True)

def compute(cloud, store=None):
    _clean()

    # write point cloud to disk in PCD format
    numpy.save(os.path.join(BASE_PATH, 'pc.npy'), cloud)
    check_call([sys.executable, 'np2pcd.py'], cwd=BASE_PATH)

    # invoke segmentation
    os.mkdir(os.path.join(BASE_PATH, 'segments'))
    check_call([os.path.join(BASE_PATH, 'region_growing_segmentation')], cwd=BASE_PATH)

    # invoke evaluation
    check_call([sys.executable, 'rate-plane.py'], cwd=BASE_PATH)

    # load the results
    grasps = []
    for result in json.load(open(os.path.join(BASE_PATH, 'planes.txt'))):
        grasps.append((
            result['score'],
            result['center'],
            result['orientation'][:3]
        ))
    grasps.sort(key=lambda x: -x[0])

    if store:
        store.put('/debug/grasps', grasps)

    _clean()

    return grasps
