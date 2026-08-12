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

# Clone px4_msgs at the commit pinned in px4_msgs.pin — the single source of
# truth for which PX4 firmware version this workspace's message definitions
# match. See px4_msgs.pin for the update procedure; a mismatch between this
# pin and the real drone's firmware makes the most critical diagnostic
# checks fail silently (CDR deserialization mismatch).
COPY px4_msgs.pin .
RUN . ./px4_msgs.pin \
    && git clone https://github.com/PX4/px4_msgs.git src/px4_msgs \
    && git -C src/px4_msgs checkout "$PX4_MSGS_COMMIT" \
    && echo "$PX4_MSGS_COMMIT" > src/px4_msgs/.pinned_commit

COPY drone_diagnostics/ src/drone_diagnostics/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select px4_msgs drone_diagnostics \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
