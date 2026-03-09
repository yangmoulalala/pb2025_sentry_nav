FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim unzip -y && \
    pip install xmacro gdown

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

# Install small_gicp
RUN apt install -y libeigen3-dev libomp-dev && \
    mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

# create workspace
RUN mkdir -p ~/ros_ws && \
    cd ~/ros_ws && \
    git clone --recursive https://github.com/yangmoulalala/pb2025_sentry_nav.git src/pb2025_sentry_nav && \
    git clone https://github.com/yangmoulalala/sentry_robot_description.git src/sentry_robot_description && \
    git clone https://github.com/yangmoulalala/mavlink_ws.git src/mavlink_ws && \
    mv src/mavlink_ws/src/rm_interfaces src/rm_interfaces && \
    rm -rf src/mavlink_ws

WORKDIR /root/ros_ws

# install dependencies and some tools
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y


# build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release


# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

# Append ROS environment to .zshrc (append to existing zsh config)
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.zsh\n\
source /root/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Also setup .bashrc for compatibility
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.bash\n\
source /root/ros_ws/install/setup.bash' >> /root/.bashrc

# Create a startup script that properly sources the workspace and runs the node
RUN echo '#!/bin/bash\n\
cd /root/ros_ws\n\
source /opt/ros/humble/setup.bash\n\
source install/setup.bash\n\
exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True "$@"' > /root/start_nav.sh && \
    chmod +x /root/start_nav.sh

# Set zsh as default shell
RUN usermod -s /bin/zsh root

RUN rm -rf /var/lib/apt/lists/*

# Use the standard ROS entrypoint and our custom startup script
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/root/start_nav.sh"]

