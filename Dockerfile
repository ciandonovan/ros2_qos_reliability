FROM ros:humble-ros-base
MAINTAINER Cian Donovan cian.donovan@akara.ai

ARG OVERLAY_WS=/opt/ros/overlay_ws
ENV OVERLAY_WS $OVERLAY_WS

# install base packages
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-$ROS_DISTRO-rmw-fastrtps-cpp \
	ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
	ros-$ROS_DISTRO-v4l2-camera \
	&& rm -rf /var/lib/apt/lists/*

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastrtps.xml
ENV CYCLONEDDS_URI=file:///cyclonedds.xml

COPY fastrtps.xml /fastrtps.xml
COPY cyclonedds.xml /cyclonedds.xml

# source ROS from .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
