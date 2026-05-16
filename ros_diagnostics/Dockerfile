FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    ros-humble-rqt-robot-monitor \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir 'influxdb-client>=1.18.0'

WORKDIR /ros2_ws

# Clone px4_msgs at the exact commit used on the drone (PX4 release/1.15)
RUN git clone https://github.com/PX4/px4_msgs.git src/px4_msgs \
    && git -C src/px4_msgs checkout a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8

COPY drone_diagnostics/ src/drone_diagnostics/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select px4_msgs drone_diagnostics \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
