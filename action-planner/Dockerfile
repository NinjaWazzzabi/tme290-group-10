FROM eigen-opencv-builder as builder

# Install libcluon
RUN add-apt-repository -y ppa:chrberger/libcluon  && \
	apt-get update && \
	apt-get install libcluon

# Add source
ADD . /opt/sources

# Add message files
WORKDIR /opt/sources

# Build
RUN mkdir /tmp/build && \
	cd /tmp/build && \
	cmake -D CMAKE_BUILD_TYPE=Release /opt/sources && \
	make && cp action-planner /tmp

# Deploy
FROM eigen-opencv-builder
RUN apt-get update
RUN mkdir /opt -p
WORKDIR /opt
COPY --from=builder /tmp/action-planner .
CMD ["/opt/action-planner"]