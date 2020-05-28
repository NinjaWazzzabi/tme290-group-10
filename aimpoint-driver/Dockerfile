FROM eigen-opencv-builder as builder

# Install c++ related packages
RUN apt-get update && \
	apt-get install -y \
		ca-certificates \
		build-essential \
		cmake \
		software-properties-common

# Install libcluon
RUN add-apt-repository -y ppa:chrberger/libcluon  && \
	apt-get update && \
	apt-get install libcluon

# Add source
ADD . /opt/sources

# Build
RUN mkdir /tmp/build && \
	cd /tmp/build && \
	cmake -D CMAKE_BUILD_TYPE=Release /opt/sources && \
	make && cp aimpoint-driver /tmp

# Deploy
FROM eigen-opencv-builder
RUN apt-get update
RUN mkdir /opt -p
WORKDIR /opt
COPY --from=builder /tmp/aimpoint-driver .
CMD ["/opt/aimpoint-driver"]