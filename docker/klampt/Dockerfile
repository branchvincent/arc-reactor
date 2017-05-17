FROM ubuntu:16.04

# install Klampt dependencies
RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    git \
    libboost-system-dev \
    libboost-thread-dev \
    freeglut3 \
    freeglut3-dev \
    libglpk-dev \
    python-dev \
    python-opengl \
    libxmu-dev \
    libxi-dev \
    libqt4-dev \
    libassimp-dev \
&& rm -rf /var/lib/apt/lists/*

# setup the Klampt build tree
RUN git clone https://github.com/krishauser/Klampt \
    && cd Klampt && git checkout v0.7

# setup the library build tree
RUN cd Klampt/Library && make unpack-deps \
    && cd KrisLibrary && git checkout plan_devel

# build library
RUN cd Klampt/Library && make deps

# install ODE library
RUN cd Klampt/Library/ode-0.14 && make install

# build Klampt
RUN cd Klampt && cmake . && make Klampt apps python

# install Klampt Python bindings
RUN cd Klampt && make python-install

# install Klampt Python GUI dependencies
RUN apt-get update && apt-get install -y \
    python-qt4 \
    python-qt4-gl \
&& rm -rf /var/lib/apt/lists/*

# clean up
RUN find Klampt | egrep "\.(tgz|gz|o)$" | xargs rm

# # install Klampt dependencies
# RUN apt-get update && apt-get install -y \
#     wget \
#     alien \
#     libboost-system1.58.0 \
#     libboost-thread1.58.0 \
#     freeglut3 \
#     libglpk36 \
#     python \
#     python-opengl \
#     libxmu6 \
#     libxi6 \
#     libassimp3v5 \
#     libtinyxml2.6.2v5 \
#     libglew1.13 \
#     libtf0d \
# && rm -rf /var/lib/apt/lists/*

# # download Klampt
# RUN wget -q http://motion.pratt.duke.edu/software/Klampt-0.7.0-Linux.deb \
#             http://motion.pratt.duke.edu/software/KrisLibrary-1.0.0-Linux.deb \
#             http://motion.pratt.duke.edu/software/Klampt-0.7.0-python2.7.x86_64.rpm

# # install Klampt
# RUN dpkg -i KrisLibrary-1.0.0-Linux.deb Klampt-0.7.0-Linux.deb && alien -i Klampt-0.7.0-python2.7.x86_64.rpm

# # clean up
# RUN apt-get remove -y alien && apt-get -y autoremove
