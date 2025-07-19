FROM ros:jazzy
ARG USERNAME=pchelichka
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	&& usermod -aG dialout $USERNAME \
	&& usermod -aG plugdev $USERNAME \
# software install
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies for building OpenCV + GStreamer
RUN sudo apt-get update && sudo apt-get install -y \
    build-essential cmake git pkg-config ros-jazzy-plotjuggler-ros \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev \
    python3-dev python3-numpy libtbbmalloc2 libtbb-dev \
    libdc1394-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
	gstreamer1.0-tools \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
	python3-serial python3-scipy \
	libusb-1.0-0 libusb-1.0-0-dev usbutils 

# Clone OpenCV and contrib (version 4.11.0)
WORKDIR /tmp
RUN git clone -b 4.11.0 https://github.com/opencv/opencv.git && \
    git clone -b 4.11.0 https://github.com/opencv/opencv_contrib.git

# Build OpenCV with GStreamer
RUN mkdir -p /tmp/opencv/build && cd /tmp/opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D WITH_TBB=ON \
          -D WITH_V4L=ON \
          -D WITH_GSTREAMER=ON \
          -D WITH_OPENGL=ON \
          -D BUILD_opencv_python3=ON \
          .. && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig

# Clean up
RUN rm -rf /tmp/opencv /tmp/opencv_contrib

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
