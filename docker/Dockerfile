# Kangaroo (PAL Robotics) MuJoCo sim + RL locomotion policy + YOLO/Jev navigation agent.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip git \
      ros-humble-mujoco-ros2-control ros-humble-mujoco-ros2-control-plugins \
      ros-humble-kangaroo-moveit-config \
      libegl1 libegl-mesa0 libgl1-mesa-dri libgles2 xvfb \
    && rm -rf /var/lib/apt/lists/*

# Upstream PAL sources, pinned.
ARG KANGAROO_ROBOT_SHA=54d539f8e9f54e9abba54000caebc7b0aa24d8bc
ARG KANGAROO_SIMULATION_SHA=3bc51f8782fad252a9e9e6212ec985c7c96fd574
# eurobin/extras: adds the 4dof locomotion baselines (v352-v357)
ARG PAL_POLICY_DEPLOYER_SHA=c1a78e8ec7d2eb799a2a284b7dc8ee9a1afefd04
WORKDIR /ws/src
RUN set -e; for spec in \
      kangaroo_robot:$KANGAROO_ROBOT_SHA \
      kangaroo_simulation:$KANGAROO_SIMULATION_SHA \
      pal_policy_deployer:$PAL_POLICY_DEPLOYER_SHA; do \
      repo=${spec%%:*}; sha=${spec##*:}; \
      git init -q $repo && git -C $repo fetch -q --depth 1 https://github.com/pal-robotics/$repo $sha \
        && git -C $repo checkout -q FETCH_HEAD; \
    done

WORKDIR /ws
RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y -r \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

# Perception + Jev client. CPU torch keeps the image small and avoids host driver coupling.
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip3 install --no-cache-dir "numpy<2" ultralytics typesafe-sdk
RUN pip3 install --no-cache-dir mujoco  # offline renders for docs/figures.py
RUN mkdir -p /opt/models && cd /opt/models \
    && python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"

ENV YOLO_MODEL=/opt/models/yolo26n-seg.pt \
    PYTHONPATH=/kangaroo_jev \
    PYTHONUNBUFFERED=1
COPY docker/entrypoint.sh /entrypoint.sh
WORKDIR /kangaroo_jev
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
