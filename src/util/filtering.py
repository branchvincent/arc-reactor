from time import time

import logging

import numpy

logger = logging.getLogger(__name__)

def distance_label(cloud, mask=None, threshold=0.005, neighborhood=4):
    threshold *= threshold
    mark = time()

    label = 0
    labeled = numpy.zeros(cloud.shape[:2], dtype=numpy.int)

    if neighborhood == 4:
        neighborhood = [ (0, -1), (-1, 0), (0, 1), (1, 0) ]
    elif neighborhood == 8:
        neighborhood = [ (0, -1), (-1, 0), (0, 1), (1, 0),
                         (-1, -1), (1, -1), (-1, 1), (1, 1) ]
    else:
        raise RuntimeError('unknown neighborhood: {}'.format(neighborhood))

    stack = []
    for r in range(cloud.shape[0]):
        for c in range(cloud.shape[1]):
            if mask[r, c]:
                stack.append((r, c))

    while stack:
        (r, c) = stack.pop()

        if not labeled[r, c]:
            label += 1
            labeled[r, c] = label
            logger.debug('created label {}'.format(label))

        # now check all the neighbors
        for (nr, nc) in [ (r + dr, c + dc) for (dr, dc) in neighborhood ]:
            if not (0 <= nr < cloud.shape[0] and 0 <= nc < cloud.shape[1]):
                continue
            if mask is not None:
                if not mask[r, c]:
                    continue

            # check if this neighbor is connected
            d = sum((cloud[r, c] - cloud[nr, nc])**2)
            if d > threshold:
                continue

            # recurse
            if not labeled[nr, nc]:
                # label the neighbor
                labeled[nr, nc] = label
                stack.append((nr, nc))

    logger.info('distance connected components in {:.3f}s'.format(time() - mark))

    return (labeled, label)
