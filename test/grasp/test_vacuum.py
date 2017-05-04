# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from unittest import TestCase

import numpy

from grasp import vacuum

class PlaneDetectionTest(TestCase):

    def setUp(self):
        try:
            import pcl
        except ImportError:
            self.skipTest('pcl is not installed')

    def test_detection(self):
        data = numpy.load('data/test/test_vacuum0.npz')

        full_cloud = data['full']
        object_clouds = [data[x] for x in data.keys() if x.startswith('arr')]

        grasps = vacuum.compute(full_cloud, object_clouds)

        self.assertIsNotNone(grasps)
        self.assertEqual(len(grasps), 3)

        for grasp in grasps:
            self.assertAlmostEqual(sum([x**2 for x in grasp[2]]), 1, places=6)
