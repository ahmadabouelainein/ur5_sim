FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
        git wget python3-pip build-essential \
        python3-vcstool python3-colcon-common-extensions \
        ros-noetic-moveit \
        && rm -rf /var/lib/apt/lists/*                                                    

WORKDIR /ws/src
RUN mkdir -p /ws/src && cd /ws/src &&\
    git clone https://github.com/ros-industrial/universal_robot.git -b noetic-devel
RUN apt-get update && rosdep update && rosdep install --from-paths /ws/src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && cd /ws && catkin_make

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]