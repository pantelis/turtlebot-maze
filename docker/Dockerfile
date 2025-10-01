# ROS distribution to use
ARG ROS_DISTRO=jazzy

#######################################
# Base Image for TurtleBot Simulation #
#######################################
FROM ros:${ROS_DISTRO} AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]


# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 curl git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine


# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create Colcon workspace with external dependencies
RUN mkdir -p /turtlebot_ws/src
WORKDIR /turtlebot_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /turtlebot_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install
ENV TURTLEBOT_MODEL=3


# Download Groot2 AppImage and place it in the home folder.
WORKDIR /root/
RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
 && chmod a+x Groot2.AppImage

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR="/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE=1

# Set up the entrypoint
WORKDIR /turtlebot_ws
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

###########################################
# Overlay Image for TurtleBot3 Simulation #
###########################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./tb3_autonomy/ ./src/tb3_autonomy/
COPY ./tb3_worlds/ ./src/tb3_worlds/
RUN source /turtlebot3_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=1000

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano sudo \
 && rm -rf /var/lib/apt/lists/*

# Create (or adapt) user/group in an idempotent way. Also fix UID/GID usage (they were reversed previously).
RUN set -eux; \
	if getent group "${GID}" > /dev/null; then \
		existing_group=$(getent group "${GID}" | cut -d: -f1); \
		if [ "${existing_group}" != "${USERNAME}" ]; then \
			groupmod -n "${USERNAME}" "${existing_group}"; \
		fi; \
	else \
		groupadd -g "${GID}" "${USERNAME}"; \
	fi; \
	if id -u "${UID}" > /dev/null 2>&1; then \
		existing_user=$(getent passwd "${UID}" | cut -d: -f1); \
		if [ "${existing_user}" != "${USERNAME}" ]; then \
			usermod -l "${USERNAME}" "${existing_user}"; \
			usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
		fi; \
	else \
		useradd -u "${UID}" -g "${GID}" -m -s /bin/bash "${USERNAME}"; \
	fi; \
	echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}; \
	chmod 0440 /etc/sudoers.d/${USERNAME}

# Ensure ownership of workspace (may already be correct if user existed)
RUN chown -R ${UID}:${GID} /overlay_ws/ || true

# Move Groot2 to new user's home directory and ensure it can be run
RUN groupadd -f fuse \
 && usermod -aG fuse ${USERNAME}
RUN mv /root/Groot2.AppImage /home/${USERNAME}/ \
 && chown ${UID}:${GID} /home/${USERNAME}/Groot2.AppImage

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
