FROM ubuntu:24.04
LABEL maintainer="kmk <kmk6061602@gmail.com>"

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
	ant \
	binutils-dev \
	ca-certificates \
	ccache \
	cmake \
	cppcheck \
	curl \
	dirmngr \
	doxygen \
	g++-multilib \
	gcc-multilib \
	gdb \
	gettext \
	git \
	gnupg \
	gosu \
	lcov \
	libelf-dev \
	libexpat-dev \
	libvecmath-java \
	libfreetype6-dev \
	libgmp-dev \
	libgtest-dev \
	libisl-dev \
	libmpc-dev \
	libmpfr-dev \
	libpng-dev \
	libssl-dev \
	lsb-release \
	make \
	ninja-build \
	openssh-client \
	openjdk-11-jre \
	openjdk-11-jdk \
	python3 \
	python3-dev \
	python3-pip \
	python3-venv \
	rsync \
	screen \
	shellcheck \
	tzdata \
	texinfo \
	u-boot-tools \
	util-linux \
	unzip \
	valgrind \
	wget \
	xsltproc \
	zip \
    sudo \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Set java 11 as default
RUN update-alternatives --set java $(update-alternatives --list java | grep "java-11")

# gtest
RUN cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

# Python 3 dependencies installed by pip
RUN python3 -m pip install --break-system-packages argparse argcomplete coverage cerberus empy==3.3.4 jinja2 kconfiglib \
    matplotlib>=3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
    pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl lxml

# manual ccache setup
RUN ln -s /usr/bin/ccache /usr/lib/ccache/cc \
	&& ln -s /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
	&& cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
	&& make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
	&& rm -rf /tmp/*

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# setup virtual X server
RUN mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix
ENV DISPLAY :99

ENV CCACHE_UMASK 000
ENV PATH "/usr/lib/ccache:$PATH"
ENV TERM xterm
ENV TZ UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    dirmngr \
    gnupg2 \
    python3-argcomplete \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    && apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install ROS2 Jazzy
ENV ROS_DISTRO jazzy

# setup ros2 keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# setup sources.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install bootstrap tools
RUN apt-get update \
	&& apt-get install --quiet -y \
		python3-colcon-common-extensions \
		python3-colcon-mixin \
		python3-rosdep \
		python3-vcstool \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# install ros2 desktop
RUN apt-get update \
	&& apt-get install --quiet --no-install-recommends -y \
		ros-$ROS_DISTRO-desktop \
		ros-$ROS_DISTRO-launch-testing-ament-cmake \
		ros-$ROS_DISTRO-ros2bag \
		ros-$ROS_DISTRO-rosidl-generator-dds-idl \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# bootstrap rosdep
RUN rosdep init && rosdep update

# setup colcon mixin and metadata
RUN colcon mixin add default \
		https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
	&& colcon mixin update \
	&& colcon metadata add default \
		https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
	&& colcon metadata update

RUN git config --global http.postBuffer 524288000

# Install Micro XRCE-DDS Agent v2.4.3
RUN git clone --branch v3.0.1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

RUN userdel -r ubuntu

WORKDIR /home/user
USER user
RUN mkdir realgazebo
WORKDIR /home/user/realgazebo

RUN git clone --branch realgazebo-1.16.0 https://github.com/SUV-Lab/RealGazebo-PX4.git --recursive \
    && git clone --branch realgazebo-1.16.0 https://github.com/SUV-Lab/RealGazebo-ROS2.git --recursive

RUN cd /home/user/realgazebo/RealGazebo-PX4 && ./Tools/setup/ubuntu.sh --no-nuttx

USER root
RUN apt-get update \
	&& apt-get install --quiet --no-install-recommends -y \
	ros-$ROS_DISTRO-ros-gz \
	ros-$ROS_DISTRO-ros-gz-sim \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER user
RUN cd /home/user/realgazebo/RealGazebo-PX4 \
   && ./Tools/download_ctrack.sh \
   && make px4_sitl \
   && cd /home/user/realgazebo/RealGazebo-ROS2 \
   && . /opt/ros/$ROS_DISTRO/setup.sh \
   && colcon build --symlink-install

USER root
# Add setup file at bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/user/.bashrc \
    && echo "source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash" >> /home/user/.bashrc \
    && chown user:user /home/user/.bashrc

# Entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]