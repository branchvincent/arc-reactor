from pensive.client import PensiveClient

s = PensiveClient().default()

mp = s.get('/robot/waypoints')

t =  [w[0] for w in mp]
q =  [w[1]['robot'][1:] for w in mp]

import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib import pyplot

import numpy
t = numpy.cumsum(t)

pyplot.figure()
pyplot.plot(t, q, '-+')

pyplot.show()
