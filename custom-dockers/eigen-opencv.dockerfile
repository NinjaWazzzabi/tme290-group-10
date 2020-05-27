FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update --fix-missing
RUN apt-get install -y \
	libgtk2.0-dev \
	pkg-config \
	cmake \
	g++ \
	make \
	git-all

# Download and install eigen lib
WORKDIR ~
run git clone https://gitlab.com/libeigen/eigen.git && cd eigen && mkdir build && cd build && cmake .. && make -j4 install

# Download and install opencv lib
WORKDIR ~
run git clone https://github.com/opencv/opencv.git && \
	cd opencv && \
	mkdir build && \
	cd build && \
	cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_GENERATE_PKGCONFIG=ON ..
run cd opencv/build && \
	make -j7
run cd opencv/build && \
	make install

# Set dynamic library path, verrrrry important
ENV LD_LIBRARY_PATH /usr/local/lib
