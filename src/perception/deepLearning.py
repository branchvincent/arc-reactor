import numpy as np
import theano
import theano.tensor as T
import lasagne
import skimage.transform
import pickle
import os
import scipy.ndimage.interpolation as sn
from scipy.misc import imresize
from lasagne.layers import InputLayer, DenseLayer, NonlinearityLayer
#from lasagne.layers.dnn import Conv2DDNNLayer as ConvLayer #requires GPU
from lasagne.layers import Conv2DLayer as ConvLayer
from lasagne.layers import Pool2DLayer as PoolLayer
from lasagne.nonlinearities import softmax
from lasagne.utils import floatX
import logging
logger = logging.getLogger(__name__)

class DeepLearningRecognizer:
    '''
    Class for performing object recognition. Uses theano and pretrained net to guess object
    '''

    def __init__(self,filename='vgg16.pkl',numclasses=1000):
        '''
        Loads in the network when class is created
        '''
        self.network = self.build_model(numclasses)
        self.weights = self.loadWeights(filename)
        self.X_sym = T.tensor4()
        lasagne.layers.set_all_param_values(self.network['prob'], self.weights['param values'])
        self.output_layer = self.network['prob']
        self.prediction = lasagne.layers.get_output(self.output_layer, self.X_sym)
        self.pred_fn = theano.function([self.X_sym], self.prediction)

    def build_model(self,numclasses):
        #NOTE: perhaps we should save the network architecture along with the weights, since they could be different
        logger.info("Building network")
        net = {}
        net['input'] = InputLayer((None, 3, 224, 224))
        net['conv1_1'] = ConvLayer(net['input'], 64, 3, pad=1)
        net['conv1_2'] = ConvLayer(net['conv1_1'], 64, 3, pad=1)
        net['pool1'] = PoolLayer(net['conv1_2'], 2)
        net['conv2_1'] = ConvLayer(net['pool1'], 128, 3, pad=1)
        net['conv2_2'] = ConvLayer(net['conv2_1'], 128, 3, pad=1)
        net['pool2'] = PoolLayer(net['conv2_2'], 2)
        net['conv3_1'] = ConvLayer(net['pool2'], 256, 3, pad=1)
        net['conv3_2'] = ConvLayer(net['conv3_1'], 256, 3, pad=1)
        net['conv3_3'] = ConvLayer(net['conv3_2'], 256, 3, pad=1)
        net['pool3'] = PoolLayer(net['conv3_3'], 2)
        net['conv4_1'] = ConvLayer(net['pool3'], 512, 3, pad=1)
        net['conv4_2'] = ConvLayer(net['conv4_1'], 512, 3, pad=1)
        net['conv4_3'] = ConvLayer(net['conv4_2'], 512, 3, pad=1)
        net['pool4'] = PoolLayer(net['conv4_3'], 2)
        net['conv5_1'] = ConvLayer(net['pool4'], 512, 3, pad=1)
        net['conv5_2'] = ConvLayer(net['conv5_1'], 512, 3, pad=1)
        net['conv5_3'] = ConvLayer(net['conv5_2'], 512, 3, pad=1)
        net['pool5'] = PoolLayer(net['conv5_3'], 2)
        net['fc6'] = DenseLayer(net['pool5'], num_units=4096)
        net['fc7'] = DenseLayer(net['fc6'], num_units=4096)
        net['fc8'] = DenseLayer(net['fc7'], num_units=numclasses, nonlinearity=None)
        net['prob'] = NonlinearityLayer(net['fc8'], softmax)
        logger.info("Network built")
        return net

    def loadWeights(self,filename):
        logger.info("Loading weights")
        # Load model weights and metadata
        d = pickle.load(open(filename, 'rb'),encoding='latin-1')
        logger.info("Weights loaded from file {}".format(filename))
        return d

    def guessObject(self, images):
        '''
        Given RGB images as np array (224x224xRGBxnum) returns a list of classes and a confidence value, and the best prediction
        For each image
        '''
        #check input
        if images.shape[0] != 224 or images.shape[1] !=224 or images.shape[2] !=3:
            logger.warning("Invalid numpy array passed to guessObject. Expecting 224x224x3xN, got {}".format(images.shape))
        
        #make the images BGR
        images = images[:, :, ::-1]

        #swap the axes so the input is Nx3x224x224
        im = np.swapaxes(images, 0, 3)
        im = np.swapaxes(im, 1, 2)
        im = np.swapaxes(im, 2, 3)

        #convert to tensor thing
        prepped_image = floatX(im)
        #guess
        guess = self.pred_fn(prepped_image)

        #guess is vector of "probabilities" guess argmax is the index of the category
        return guess