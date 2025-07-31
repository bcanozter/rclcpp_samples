FROM ros:jazzy-ros-core
ENV ROS_DISTRO=jazzy

RUN apt-get update -y && apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    make

RUN apt-get clean
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "cd /home/container/" >> ~/.bashrc
COPY --chmod=755 ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]