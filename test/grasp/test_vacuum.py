# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from unittest import TestCase

import numpy
import scipy.ndimage
from math import tan, pi, sqrt

from grasp import vacuum

from master.world import xyz, rpy
from util.math_helpers import transform

def _call_with_self(target, *args, **kwargs):
    def _cb(self):
        return target(self, *args, **kwargs)
    return _cb

def _generate_pick_tests():
    def setUp(self):
        try:
            import pcl
        except ImportError:
            self.skipTest('pcl is not installed')

    def raycast(projection, planes, resolution):
        [u, v] = numpy.meshgrid(
            numpy.linspace(-1, 1, resolution[0]),
            numpy.linspace(-1, 1, resolution[1])
        )

        clip = numpy.dstack((
            u,
            v,
            numpy.full(u.shape, -1),
            numpy.full(u.shape, 1)
        ))
        eye = transform(projection, clip)
        world = numpy.dstack((eye[:, :, 0], eye[:, :, 1], numpy.full(u.shape, -1)))
        rays = world / numpy.linalg.norm(world, axis=2, keepdims=True)

        distance = numpy.full(u.shape, float('Inf'))
        labels = numpy.full(u.shape, 0, dtype=numpy.int)

        for (idx, (pose, size)) in enumerate(planes):
            pose = numpy.asarray(pose)

            origin = pose[:3, 3]
            normal = pose[:3, :3].dot([[0], [0], [1]]).T

            # propagation distance
            d = numpy.sum(origin * normal) / numpy.sum(rays * [normal], axis=2)

            # check if intersection within plane size
            intersection = transform(numpy.linalg.inv(pose), d[:, :, numpy.newaxis] * rays)

            mask = (d > 0)

            for dim in [0, 1]:
                mask = numpy.logical_and(mask, numpy.absolute(intersection[:, :, dim]) <= (size[dim] / 2.0))

            update = numpy.logical_and(mask, d < distance)
            distance[update] = d[update]
            labels[update] = idx + 1

        valid = (distance != float('Inf'))

        # apply depth blurring
        #distance = scipy.ndimage.filters.gaussian_filter(distance, 1)

        # apply gaussian noise
        distance[valid] += numpy.random.normal(0, 0.005 * distance[valid], distance[valid].shape)

        distance[~valid] = 0

        cloud = distance[:, :, numpy.newaxis] * rays

        from matplotlib import cm
        labels_rgb = 255 * numpy.array(cm.Set1.colors)[(labels - 1) % len(cm.Set1.colors)]
        labels_rgb[labels == 0] = [0, 0, 0]

        from util import pcd
        pcd.write(zip(cloud[valid].tolist(), labels_rgb[valid].tolist()), '/tmp/test.pcd')

        import cv2
        cv2.imwrite('/tmp/labels.png', labels_rgb[:, :, ::-1])

        return (cloud, labels)

    def handle_test(self, planes):
        n = 0.1
        f = 2

        fov_width = 68
        fov_height = 54

        w = 2 * n * tan(pi / 180 * fov_width / 2)
        h = 2 * n * tan(pi / 180 * fov_height / 2)

        projection = numpy.array([
            [n/h, 0, 0, 0],
            [0, n/w, 0, 0],
            [0, 0, -(f+n)/(f-n), -2*f*n/(f-n)],
            [0, 0, -1, 0]
        ])

        (full_cloud, object_mask) = raycast(projection, planes, (640, 480))
        object_clouds = [ full_cloud[object_mask == (idx + 1)] for idx in range(object_mask.max())]

        grasps = vacuum.compute(full_cloud, object_clouds)

        self.assertIsNotNone(grasps)
        self.assertEqual(len(grasps), len(object_clouds))

        for ((pose, size), (score, center, orientation)) in zip(planes, grasps):
            pose = numpy.asarray(pose)

            local_center = transform(numpy.linalg.inv(pose), center).flat

            for dim in [0, 1]:
                self.assertLessEqual(abs(local_center[dim]), size[dim] / 2.0)

    methods = {
        'setUp': setUp,
        'raycast': raycast,
    }

    # density = 1

    # for bin_index in range(3):
    #     for point_index in range(density**3):
    #         for box_index in range(3):
    #             methods['test_pick_{:03d}_{:03d}_{:03d}'.format(bin_index, point_index, box_index)] = _call_with_self(handle_test, bin_index, density, point_index, box_index)

    methods['test_grasp'] = _call_with_self(handle_test, [(xyz(0, 0, -1), (0.5, 0.5)), (xyz(0, 0, -0.75) * rpy(pi/4, 0, 0), (0.1, 0.2)), (xyz(0.25, 0, -0.75) * rpy(pi/4, pi/4, 0), (0.5, 0.5))])

    return type('VacuumGraspTest', (TestCase,), methods)

VacuumGraspTest = _generate_pick_tests()
