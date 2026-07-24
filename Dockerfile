FROM osrf/ros:noetic-desktop-full
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
# Non-root user so start.sh can run the container as a normal user (matches
# the --user=px4devel:px4devel flag there) instead of root, while still
# allowing passwordless sudo for the few steps that need it (below, and
# interactively inside the container).
RUN useradd -ms /bin/bash px4devel
RUN usermod -aG sudo px4devel
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR home/px4devel/
USER px4devel
# --recursive: PX4-Autopilot pulls in its own submodules (simulators,
# board configs, etc.) that the build won't work without.
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# Empty catkin workspace baked into the image; start.sh bind-mounts the
# real catkin_ws/ over this at runtime, so this just ensures the mount
# point and ownership exist beforehand.
RUN mkdir -p catkin_ws/src
RUN chown -R px4devel:px4devel catkin_ws
# MAVROS needs GeographicLib's geoid/datum datasets for local<->global
# position conversions; they're not bundled with the apt package.
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x ./install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh
RUN echo "source /opt/ros/noetic/setup.bash" >> .bashrc
