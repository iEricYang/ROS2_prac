# 使用 ROS 的镜像
FROM osrf/ros:humble-desktop

# 安装软件包
RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        ros-humble-gazebo-* \
        ros-humble-turtle-tf2-py \
        ros-humble-tf2-tools \
        ros-humble-tf-transformations\
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 包
RUN pip3 install tornado simplejpeg transforms3d

# 设置容器的主命令为bash终端
#CMD ["/bin/bash"]