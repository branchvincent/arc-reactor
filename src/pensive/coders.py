'''
JSON encoders/decoders for specific complex types.
'''

from base64 import b64encode, b64decode

from .client import JSON_DECODERS, JSON_ENCODERS

def register_numpy():
    '''
    Register coders for `numpy.ndarray` and `numpy.ndmatrix`.

    To use these coders, Python dictionaries must not contain the keys
    `__numpy.ndarray__` or `__numpy.matrix__` in routine data.

    Returns `True` on success and `False` otherwise.
    '''

    try:
        # attempt to import numpy
        import numpy
    except ImportError:
        return False

    # base encoder for ndarray and matrix
    def encode(obj):
        # ref: http://stackoverflow.com/questions/3488934/simplejson-and-numpy-array
        if obj.flags['C_CONTIGUOUS']:
            data = obj.data
        else:
            data = numpy.ascontiguousarray(obj).data

        return {
            'data': b64encode(data),
            'dtype': str(obj.dtype),
            'shape': obj.shape,
        }

    # install encoding handlers
    JSON_ENCODERS[numpy.ndarray] = lambda obj: {'__numpy.ndarray__': encode(obj)}
    JSON_ENCODERS[numpy.matrix] = lambda obj: {'__numpy.matrix__': encode(obj)}

    def decode_ndarray(obj):
        return numpy.frombuffer(b64decode(obj['data']), obj['dtype']).reshape(obj['shape'])

    # install decoding handlers
    JSON_DECODERS['__numpy.ndarray__'] = decode_ndarray
    JSON_DECODERS['__numpy.matrix__'] = lambda obj: numpy.asmatrix(decode_ndarray(obj))

    return True
