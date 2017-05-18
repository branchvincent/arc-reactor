FROM klampt:0.7

# install core dependencies
RUN apt-get update && apt-get install -y \
    wget \
    git \
    python-pip \
    python-opencv \
    python-numpy \
    python-scipy \
    python-pyside \
    python-qt4 \
    python-qt4-gl \
    libpcl-dev \
&& rm -rf /var/lib/apt/lists/*

# build python-pcl
RUN git clone https://github.com/strawlab/python-pcl && cd python-pcl \
    && pip install cython && make

# install python-pcl
RUN mkdir -p /usr/lib/python2.7/dist-packages/pcl \
    && cp python-pcl/pcl/*.py /usr/lib/python2.7/dist-packages/pcl \
    && cp python-pcl/pcl/*.so /usr/lib/python2.7/dist-packages/pcl \
    && rm -rf python-pcl
