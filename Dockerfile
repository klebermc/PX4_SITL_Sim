FROM osrf/ros:noetic-desktop-full
LABEL maintainer="patrick.lavallee@drdc-rddc.gc.ca"
LABEL version="0.1"
LABEL description="This is custom Docker Image for \
building PX4 with ROS/Gazebo compatibility for SITL."
RUN apt-get update && apt-get install -y \
	apt-utils \
	mesa-utils \
	git \
	xauth \
	wget \
	nano \
	python3-catkin-tools \
	python3-tk \
	python3-dbg \
	ros-noetic-mavros-msgs \
	ros-noetic-mavros \
	ros-noetic-mavros-extras
RUN useradd -ms /bin/bash px4devel
RUN usermod -aG sudo px4devel
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR home/px4devel/
USER px4devel
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
RUN mkdir -p catkin_ws/src
RUN chown -R px4devel:px4devel catkin_ws
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x ./install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh
RUN echo "source /opt/ros/noetic/setup.bash" >> .bashrc
