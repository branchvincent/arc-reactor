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
        # assume a column vector
        array = numpy.array([array]).T

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
