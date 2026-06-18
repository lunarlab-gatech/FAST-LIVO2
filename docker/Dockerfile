# Dockerfile for FAST-LIVO2 (ROS Noetic, Ubuntu 20.04)
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    tmux tmuxp fonts-dejavu-core \
    sudo && \
    rm -rf /var/lib/apt/lists/*

# Install Sophus at commit a621ff (non-templated/double-only version per README)
# Patch so2.cpp: a621ff uses C++98-style complex assignment rejected by GCC 9 (C++14 default)
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    git checkout a621ff && \
    sed -i 's/unit_complex_.real() = 1\./unit_complex_.real(1.)/g' sophus/so2.cpp && \
    sed -i 's/unit_complex_.imag() = 0\./unit_complex_.imag(0.)/g' sophus/so2.cpp && \
    mkdir build && cd build && cmake .. && \
    make -j$(nproc) && \
    make install && \
    mkdir -p /usr/local/lib/cmake/Sophus && \
    printf 'set(Sophus_FOUND TRUE)\nset(SOPHUS_INCLUDE_DIRS /usr/local/include)\nset(Sophus_INCLUDE_DIRS /usr/local/include)\nset(SOPHUS_LIBRARIES /usr/local/lib/libSophus.so)\nset(Sophus_LIBRARIES /usr/local/lib/libSophus.so)\n' \
        > /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    rm -rf /tmp/Sophus

# Set default user arguments which should be overwritten by docker build
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

# Create a non-root user with matching UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    groupadd --gid 110 render && \
    usermod -aG render ${USERNAME}

# Add a custom .bashrc with warm colors and fixed hostname
RUN echo 'export PS1="\\[\\e[1;31m\\]\\u@fast_livo2_container\\[\\e[0m\\]:\\[\\e[1;33m\\]\\w\\[\\e[0m\\]\\$ "' > /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

# Set default user & shell
USER ${USERNAME}
WORKDIR /home/${USERNAME}
SHELL ["/bin/bash", "-c"]
RUN echo "export USER=${USERNAME}" >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN echo "set -g mouse on" >> /home/${USERNAME}/.tmux.conf
