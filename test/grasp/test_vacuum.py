# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from unittest import TestCase

import numpy

from grasp import vacuum

class PlaneDetectionTest(TestCase):

    def setUp(self):
        self.pc = numpy.load('data/simulation/pc-stow-0.npy')

    def test_detection(self):
        grasps = vacuum.compute(self.pc)

        self.assertIsNotNone(grasps)
        self.assertEqual(len(grasps), 3)

        for grasp in grasps:
            self.assertAlmostEqual(sum([x**2 for x in grasp[2]]), 1, places=6)
