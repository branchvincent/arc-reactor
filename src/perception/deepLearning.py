import numpy as np
import theano
import theano.tensor as T
import lasagne
import pickle
import os
import lasagne
from lasagne.layers import InputLayer
from lasagne.layers import Conv2DLayer as ConvLayer
from lasagne.layers import BatchNormLayer
from lasagne.layers import Pool2DLayer as PoolLayer
from lasagne.layers import NonlinearityLayer
from lasagne.layers import ElemwiseSumLayer
from lasagne.layers import DenseLayer
from lasagne.nonlinearities import rectify, softmax
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
        lasagne.layers.set_all_param_values(self.network['prob'], self.weights['values'])
        self.output_layer = self.network['prob']
        self.prediction = lasagne.layers.get_output(self.output_layer, self.X_sym)
        self.pred_fn = theano.function([self.X_sym], self.prediction)

    def build_model(self, num_classes):
        logger.info("Building model...")
        net = {}
        net['input'] = InputLayer((None, 3, 224, 224))
        sub_net, parent_layer_name = build_simple_block(
            net['input'], ['conv1', 'bn_conv1', 'conv1_relu'],
            64, 7, 2, 3, use_bias=True)
        net.update(sub_net)
        net['pool1'] = PoolLayer(net[parent_layer_name], pool_size=3, stride=2, pad=0, mode='max', ignore_border=False)
        block_size = list('abc')
        parent_layer_name = 'pool1'
        for c in block_size:
            if c == 'a':
                sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1, 1, True, 4, ix='2%s' % c)
            else:
                sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='2%s' % c)
            net.update(sub_net)

        block_size = list('abcd')
        for c in block_size:
            if c == 'a':
                sub_net, parent_layer_name = build_residual_block(
                    net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='3%s' % c)
            else:
                sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='3%s' % c)
            net.update(sub_net)

        block_size = list('abcdef')
        for c in block_size:
            if c == 'a':
                sub_net, parent_layer_name = build_residual_block(
                    net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='4%s' % c)
            else:
                sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='4%s' % c)
            net.update(sub_net)

        block_size = list('abc')
        for c in block_size:
            if c == 'a':
                sub_net, parent_layer_name = build_residual_block(
                    net[parent_layer_name], 1.0/2, 1.0/2, True, 4, ix='5%s' % c)
            else:
                sub_net, parent_layer_name = build_residual_block(net[parent_layer_name], 1.0/4, 1, False, 4, ix='5%s' % c)
            net.update(sub_net)
        net['pool5'] = PoolLayer(net[parent_layer_name], pool_size=7, stride=1, pad=0,
                                mode='average_exc_pad', ignore_border=False)
        net['fc1000'] = DenseLayer(net['pool5'], num_units=num_classes, nonlinearity=None)
        net['prob'] = NonlinearityLayer(net['fc1000'], nonlinearity=softmax)
        logger.info("Model built")
        return net

    def loadWeights(self,filename):
        logger.info("Loading weights from {}".format(filename))
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


def build_simple_block(incoming_layer, names,
                    num_filters, filter_size, stride, pad,
                    use_bias=False, nonlin=rectify):
    """Creates stacked Lasagne layers ConvLayer -> BN -> (ReLu)
    Parameters:
    ----------
    incoming_layer : instance of Lasagne layer
        Parent layer
    names : list of string
        Names of the layers in block
    num_filters : int
        Number of filters in convolution layer
    filter_size : int
        Size of filters in convolution layer
    stride : int
        Stride of convolution layer
    pad : int
        Padding of convolution layer
    use_bias : bool
        Whether to use bias in conlovution layer
    nonlin : function
        Nonlinearity type of Nonlinearity layer
    Returns
    -------
    tuple: (net, last_layer_name)
        net : dict
            Dictionary with stacked layers
        last_layer_name : string
            Last layer name
    """
    net = []
    net.append((
            names[0],
            ConvLayer(incoming_layer, num_filters, filter_size, stride, pad,
                    flip_filters=False, nonlinearity=None) if use_bias
            else ConvLayer(incoming_layer, num_filters, filter_size, stride, pad, b=None,
                        flip_filters=False, nonlinearity=None)
        ))

    net.append((
            names[1],
            BatchNormLayer(net[-1][1])
        ))
    if nonlin is not None:
        net.append((
            names[2],
            NonlinearityLayer(net[-1][1], nonlinearity=nonlin)
        ))

    return dict(net), net[-1][0]


def build_residual_block(incoming_layer, ratio_n_filter=1.0, ratio_size=1.0, has_left_branch=False,
                        upscale_factor=4, ix=''):
    """Creates two-branch residual block
    Parameters:
    ----------
    incoming_layer : instance of Lasagne layer
        Parent layer
    ratio_n_filter : float
        Scale factor of filter bank at the input of residual block
    ratio_size : float
        Scale factor of filter size
    has_left_branch : bool
        if True, then left branch contains simple block
    upscale_factor : float
        Scale factor of filter bank at the output of residual block
    ix : int
        Id of residual block
    Returns
    -------
    tuple: (net, last_layer_name)
        net : dict
            Dictionary with stacked layers
        last_layer_name : string
            Last layer name
    """
    simple_block_name_pattern = ['res%s_branch%i%s', 'bn%s_branch%i%s', 'res%s_branch%i%s_relu']

    net = {}

    # right branch
    net_tmp, last_layer_name = build_simple_block(
        incoming_layer, list(map(lambda s: s % (ix, 2, 'a'), simple_block_name_pattern)),
        int(lasagne.layers.get_output_shape(incoming_layer)[1]*ratio_n_filter), 1, int(1.0/ratio_size), 0)
    net.update(net_tmp)

    net_tmp, last_layer_name = build_simple_block(
        net[last_layer_name], list(map(lambda s: s % (ix, 2, 'b'), simple_block_name_pattern)),
        lasagne.layers.get_output_shape(net[last_layer_name])[1], 3, 1, 1)
    net.update(net_tmp)

    net_tmp, last_layer_name = build_simple_block(
        net[last_layer_name], list(map(lambda s: s % (ix, 2, 'c'), simple_block_name_pattern)),
        lasagne.layers.get_output_shape(net[last_layer_name])[1]*upscale_factor, 1, 1, 0,
        nonlin=None)
    net.update(net_tmp)

    right_tail = net[last_layer_name]
    left_tail = incoming_layer

    # left branch
    if has_left_branch:
        net_tmp, last_layer_name = build_simple_block(
            incoming_layer, list(map(lambda s: s % (ix, 1, ''), simple_block_name_pattern)),
            int(lasagne.layers.get_output_shape(incoming_layer)[1]*4*ratio_n_filter), 1, int(1.0/ratio_size), 0,
            nonlin=None)
        net.update(net_tmp)
        left_tail = net[last_layer_name]

    net['res%s' % ix] = ElemwiseSumLayer([left_tail, right_tail], coeffs=1)
    net['res%s_relu' % ix] = NonlinearityLayer(net['res%s' % ix], nonlinearity=rectify)

    return net, 'res%s_relu' % ix