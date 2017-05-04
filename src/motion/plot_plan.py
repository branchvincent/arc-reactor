import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib import pyplot

import numpy
from math import pi

from pensive.client import PensiveClient

s = PensiveClient().default()

mp = s.get('/robot/waypoints')

t =  numpy.cumsum([w[0] for w in mp])
q =  numpy.array([w[1]['robot'][1:] for w in mp])
v =  numpy.array([w[1]['vacuum'][0] for w in mp])

pyplot.figure()
pyplot.plot(t, q * 180 / pi, '-+')
pyplot.plot(t, v * 100, '-+')

pyplot.xlabel('time (s)')
pyplot.ylabel('value')
pyplot.legend(['q{} (deg)'.format(i + 1) for i in range(len(q[0]))] + ['vacuum (%)'], bbox_to_anchor=(1,1))

pyplot.grid(True)
pyplot.show()