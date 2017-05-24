import numpy
import cv2
import matplotlib
matplotlib.use('Qt4Agg')

#...EMInit OK....EMRunMode:SOFTWARE_TRIGGER....EMRunMode OK....EMStart....Start OK....CheckOfUpdateFirmware'...CheckOfUpdateFirmware NoUpdateRequested
#...EMGetSettings....

from matplotlib import pyplot

data = open('/tmp/dump.bin', 'rb').read()

width = 2064
height = 1544

#import pdb; pdb.set_trace();

depth = numpy.frombuffer(data, dtype='>f', count=width*height, offset=0x4e + 2 + width).reshape(height, width)
#depth = depth[::-1, ::-1]

pyplot.figure()
pyplot.imshow(depth.clip(0, 3000), cmap='viridis', interpolation='none')
pyplot.colorbar()
pyplot.title('Depth')

normals = numpy.frombuffer(data, dtype='u1', count=2*width*height, offset=0x4e + 3*width*height + 2*384*width - width).reshape(height, width, 2)
#normals = normals[::-1, ::-1, :]

pyplot.figure()
pyplot.imshow(normals[:, :, 0], cmap='viridis', interpolation='none')
pyplot.colorbar()
pyplot.title('Normals X')

pyplot.figure()
pyplot.imshow(normals[:, :, 1], cmap='viridis', interpolation='none')
pyplot.colorbar()
pyplot.title('Normals Y')

texture = numpy.frombuffer(data, dtype='u2', count=width*height, offset=0x4e + 5*width*height + 2*384*width - width).reshape(height, width)
#texture = texture[::-1, ::-1]

pyplot.figure()
pyplot.imshow(texture.clip(0, 512), cmap='viridis', interpolation='none')
pyplot.colorbar()
pyplot.title('Texture')


# for i in range(6):
#     for offset in range(2):
#         img = numpy.frombuffer(data, dtype='u2', count=width*height, offset=0x25 + offset + i*width*height)
#         img = img.reshape((height, width))
# #img2 = img2.reshape((height, width))

#     #import pdb; pdb.set_trace();

# #import pdb; pdb.set_trace();


# #img = numpy.clip(img, -1, 1)
# #img2 = numpy.clip(img2, -5, 5)

#         pyplot.figure()
#         pyplot.imshow(img[:,:], cmap='viridis')
#         pyplot.title('image {}, offset {}'.format(i, offset))
#         pyplot.colorbar()

# pyplot.figure()
# pyplot.imshow(img2, cmap='viridis')
# pyplot.colorbar()



pyplot.show()
