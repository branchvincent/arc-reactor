'''
Helper functions for transforming point clouds and 3-vector arrays.
'''

import numpy

def transform(pose, array, row=None):
    '''
    Apply a rigid body transform `pose` to `array`.

    `pose` is a 4x4 NumPy array or matrix or a list.
    `array` is a NumPy array or matrix or a list of at least 2 dimensions.

    If `array` has two dimensions, row or column layout using `row` argument.
    If `row is None`, the layout is guessed by checking if the first or second
    dimension has length 3. Square arrays are not supported.BaseException

    If `array` has more than two dimensions, the last dimension is transformed
    and must have length 3.
    '''

    # ensure conversion to NumPy types without copying existing arrays
    array = numpy.asarray(array)
    pose = numpy.asarray(pose)

    if len(array.shape) == 1 and array.shape[0] in [3, 4]:
        # assume a row vector
        array = numpy.array([array])

    if len(array.shape) > 2:
        # assume last dimension is to be transformed for multidimensional array
        if array.shape[-1] == 3:
            # apply transformation
            return (array.reshape((-1, 3)).dot(pose[:3, :3].T) + pose[:3, 3].T).reshape(array.shape)
        elif array.shape[-1] == 4:
            # apply transformation
            return array.reshape((-1, 4)).dot(pose.T).reshape(array.shape)
        else:
            raise RuntimeError('last dimension not of length 3 or 4 for transform: {}'.format(array.shape))

    else:
        if row is None:
            if array.shape == (3, 3):
                raise RuntimeError('cannot detect row or column layout for 3x3 array: {}'.format(array.shape))

            row = (array.shape[1] == 3)

        if row:
            return array.dot(pose[:3, :3].T) + pose[:3, 3].T
        else:
            return pose[:3, :3].dot(array) + pose[:3, 3]


def rotate(pose, array, **kwargs):
    '''
    Apply a rigid body rotation `pose` to `array`.
    '''

    # ensure conversion to NumPy types without copying existing arrays
    pose = numpy.asarray(pose).copy()

    # suppress translation
    pose[:3, 3] = 0

    return transform(pose, array, **kwargs)

def normalize(vector):
    '''
    Normalize a vector
    '''

    vector = numpy.array(vector)
    return vector / ((vector**2).sum())**0.5

def build_pose(store, urls, strict=True):
    '''
    Multiply poses together from multiple database URLs.

    If `strict`, a database error is thrown if a URL does not exist.
    Otherwise, `None` is returned.
    '''

    full_pose = numpy.eye(4)
    for url in urls:
        pose = store.get(url, strict=strict)
        if pose is None:
            return None
        else:
            full_pose = full_pose.dot(pose)

    return full_pose

def crop_with_aabb(cloud, aabb, mask=None):
    if mask is None:
        mask = numpy.full((cloud.shape[:-1]), True)

    if cloud.shape[-1] != len(aabb[0]):
        raise RuntimeError('point cloud and AABB dimensionality mismatch: {} vs {}'.format(cloud.shape, len(aabb[0])))

    for dim in range(cloud.shape[-1]):
        mask = numpy.logical_and(mask, cloud[..., dim] >= min([aabb[0][dim], aabb[1][dim]]))
        mask = numpy.logical_and(mask, cloud[..., dim] <= max([aabb[0][dim], aabb[1][dim]]))

    return mask
