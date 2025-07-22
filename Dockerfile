FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      git wget build-essential bash-completion \
      python3-pip python3-rosdep \
      python3-vcstool python3-colcon-common-extensions \
      ros-noetic-moveit \
      ros-noetic-universal-robots \
      ros-noetic-rosbash \
      && rm -rf /var/lib/apt/lists/*

ENV WS=/ur5_sim/ws
WORKDIR ${WS}

RUN pip3 install --no-cache-dir ollama llama-cpp-python termcolor

RUN mkdir -p ${WS}/src && \
    cd ${WS}/src && \
    git init universal_robot && \
    cd universal_robot && \
    git remote add origin https://github.com/ros-industrial/universal_robot.git && \
    git sparse-checkout init --cone && \
    git sparse-checkout set ur_kinematics && \
    git pull origin noetic-devel

RUN rosdep update && \
    rosdep install --from-paths ${WS}/src --ignore-src -r -y && \
    source /opt/ros/noetic/setup.bash && \
    cd ${WS} && catkin_make


COPY docker/*.sh /
RUN chmod +x /*.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
