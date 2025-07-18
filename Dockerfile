FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
        git wget python3-pip build-essential \
        python3-vcstool python3-colcon-common-extensions \
        && rm -rf /var/lib/apt/lists/*                                                    
RUN apt-get update && apt-get install -y \
        ros-noetic-universal-robots \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-gazebo-ros-control \
        && rm -rf /var/lib/apt/lists/*                                                    

WORKDIR /workspace
RUN mkdir -p /workspace/src
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]   # default when you `docker compose run`
