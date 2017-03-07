FROM klampt:0.7

# install core dependencies
RUN apt-get update && apt-get install -y \
    wget \
    python-pip \
    python-opencv \
    python-numpy \
    python-scipy \
    python-pyside \
    python-qt4 \
    python-qt4-gl \
&& rm -rf /var/lib/apt/lists/*
