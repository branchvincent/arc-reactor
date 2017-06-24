from time import time

import numpy

from pensive.client import PensiveClient

def run():
    s = PensiveClient().default()

    data = numpy.random.rand(640, 480, 3).astype(numpy.float64)

    n = 10
    mark = time()
    for i in range(n):
        s.put('/tmp/test', data)
        assert numpy.allclose(s.get('/tmp/test'), data)
    duration = time() - mark
    print(duration, duration / n)

if __name__ == '__main__':
    run()
