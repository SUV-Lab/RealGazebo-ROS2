FROM px4io/px4-dev-simulation-jammy
LABEL maintainer="kmk <kmk6061602@gmail.com>"

ENV ROS_DISTRO=humble

RUN apt update && apt install -y \
    git \
    sudo

RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# install ros2 humble
RUN apt update && apt install -y locales \
   && locale-gen en_US en_US.UTF-8 \
   && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
   && export LANG=en_US.UTF-8 \
   && apt install -y software-properties-common \
   && add-apt-repository universe \
   && apt update && apt install curl -y \
   && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
   && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
   && apt update && apt upgrade -y \
   && apt install -y ros-$ROS_DISTRO-desktop \
   python3-colcon-common-extensions \
   && apt -y autoremove \
   && apt clean autoclean \
   && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN git config --global http.postBuffer 524288000

# Install Micro XRCE-DDS Agent v3.0.1
RUN git clone --branch v3.0.1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install && ldconfig \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

WORKDIR /home/user
USER user
RUN mkdir realgazebo
WORKDIR /home/user/realgazebo

RUN git clone --branch realgazebo-1.14 https://github.com/SUV-Lab/RealGazebo-PX4-Autopilot.git --recursive \
    && git clone https://github.com/SUV-Lab/RealGazebo-ROS2.git --recursive

RUN cd /home/user/realgazebo/RealGazebo-PX4-Autopilot && ./Tools/setup/ubuntu.sh --no-nuttx

USER root
RUN curl -sSL http://get.gazebosim.org | sh
RUN apt update \
   && apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs \
   ros-$ROS_DISTRO-gazebo-ros \
   xterm \
   && apt-get -y autoremove \
   && apt-get clean autoclean \
   && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN pip3 install gdown

USER user
RUN cd /home/user/realgazebo/RealGazebo-PX4-Autopilot \
   && ./Tools/download_ctrack.sh \
   && DONT_RUN=1 make px4_sitl_default gazebo-classic \
   && cd /home/user/realgazebo/RealGazebo-ROS2 \
   && . /opt/ros/humble/setup.sh \
   && colcon build --symlink-install

USER root
# Add setup file at bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/user/.bashrc \
    && echo "source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash" >> /home/user/.bashrc \
    && chown user:user /home/user/.bashrc

# Entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]