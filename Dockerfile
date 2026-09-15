# Noetic 的官方 ros-base 镜像使用 Ubuntu 20.04 Focal userspace。
FROM ros:noetic-ros-base-focal

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# 安装构建、调试、源码阅读和基础诊断工具。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        gdb \
        gdbserver \
        doxygen \
        graphviz \
        pkg-config \
        python3-catkin-tools \
        python3-rosdep \
        tree \
        less \
        vim \
        procps \
        iproute2 \
        iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 创建普通用户，并让 UID/GID 与 Host 一致。
RUN if ! getent group "${USER_GID}" >/dev/null 2>&1; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi && \
    useradd \
        --uid "${USER_UID}" \
        --gid "${USER_GID}" \
        --create-home \
        --shell /bin/bash \
        "${USERNAME}"

# Dev Container 会把 VS Code Server 安装到这个目录。
RUN mkdir -p "/home/${USERNAME}/.vscode-server" && \
    chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}/.vscode-server"

# 每个交互式 bash 自动加载同一份 ROS 环境脚本。
RUN printf '%s\n' \
    'if [ -f /workspace/.devcontainer/ros_env.bash ]; then' \
    '    source /workspace/.devcontainer/ros_env.bash' \
    'else' \
    '    source /opt/ros/noetic/setup.bash' \
    'fi' \
    >> "/home/${USERNAME}/.bashrc"

USER ${USERNAME}
WORKDIR /workspace

# compose.yaml 会用 sleep infinity 覆盖它；单独 docker run 时仍可进入 bash。
CMD ["bash"]
