# ROS 2 Jazzy + Gazebo Harmonic image for the mining_sim_gz package (build context = the gazebo/ folder).
#   docker build -t mining-sim-gz -f docker/Dockerfile .
FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz python3-scipy python3-pytest libegl1 libgl1-mesa-dri xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros2_ws/src ./src
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --packages-select mining_sim_gz

COPY docker/run_headless.sh /usr/local/bin/run_headless.sh
RUN chmod +x /usr/local/bin/run_headless.sh
# software rendering (llvmpipe) so the gpu_lidar works without a GPU
ENV LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
CMD ["bash"]
