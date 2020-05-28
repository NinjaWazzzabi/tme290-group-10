FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update --fix-missing
RUN apt-get install -y \
	libgtk2.0-dev \
	pkg-config \
	cmake \
	g++ \
	make \
	git-all \
	ca-certificates \
	build-essential \
	software-properties-common

# Install pre-compiled opencv & eigen files
COPY lib /usr/local/lib
COPY include /usr/local/include

# WORKDIR ~
# run git clone https://gitlab.com/libeigen/eigen.git && cd eigen && mkdir build && cd build && cmake .. && make -j4 install

# Set dynamic library path, verrrrry important
ENV LD_LIBRARY_PATH /usr/local/lib
