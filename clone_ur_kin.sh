cd ws/src
git clone --depth 1 --filter=blob:none --sparse \
          --branch noetic-devel \
          https://github.com/ros-industrial/universal_robot.git

cd universal_robot
git sparse-checkout set ur_kinematics