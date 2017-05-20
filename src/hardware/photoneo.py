import socket
import sys
import struct

from time import sleep

def readall(s, n):
    data = ''
    while len(data) < n:
        chunk = s.recv(n - len(data))
        if len(chunk) == 0:
            return None
        else:
            data += chunk

    return data

def read_string(s, length=None):
    if length is None:
        data = readall(s, 4)
        (length,) = struct.unpack('<I', data)

    return readall(s, length)

def read_int(s, signed=False):
    data = readall(s, 4)
    (length,) = struct.unpack('<I', data)

    if length <= 0:
        return None

    fmt = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}[length]
    if signed:
        fmt = fmt.lower()

    data = readall(s, length)
    (number,) = struct.unpack('<{}'.format(fmt), data)

    return number

def write_string(s, data):
    s.sendall(struct.pack('<I', len(data)) + data)

if __name__ == '__main__':
    address = sys.argv[1]
    port = int(sys.argv[2])

    app_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # handshake
    app_socket.connect((address, port))
    write_string(app_socket, 'I\'m ClientSideApp')
    print read_string(app_socket)
    write_string(app_socket, 'adminMode')
    print read_string(app_socket)
    write_string(app_socket, 'EMInit')

    sleep(1)

    # open data datastream
    data_socket.connect((address, port + 35))
    write_string(data_socket, 'I\'m DataStream')
    print read_string(data_socket)

    # initialization
    print read_string(app_socket)
    length = read_int(app_socket)
    readall(app_socket, 4)
    config = read_string(app_socket, length)
    print config
    print read_string(app_socket)

    # software trigger
    write_string(app_socket, 'EMSoftTrig')
    print read_string(app_socket)

    app_socket.close()
    data_socket.close()
