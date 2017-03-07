FROM klampt:0.7

# install core dependencies
RUN apt-get update && apt-get install -y \
    python-opencv \
    python-numpy \
    python-scipy \
    python-pip \
&& rm -rf /var/lib/apt/lists/*
