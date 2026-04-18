FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    ros-humble-rqt-robot-monitor \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir 'influxdb-client>=1.18.0'

WORKDIR /ros2_ws

# px4_msgs injected via additional_contexts in docker-compose
COPY --from=px4_msgs . src/px4_msgs/
COPY drone_diagnostics/ src/drone_diagnostics/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select px4_msgs drone_diagnostics \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
