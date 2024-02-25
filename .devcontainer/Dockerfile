ARG CUDA_VERSION=12.2.2
ARG CUDNN_VERSION=8
ARG OS_VERSION=20.04

FROM osrf/ros:noetic-desktop-full as ros
WORKDIR /ROS

# 拉取基础镜像
FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${OS_VERSION}
WORKDIR /root

COPY --from=ros /opt/ros/noetic /opt/ros/noetic
COPY --from=ros /etc/ros /etc/ros

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

LABEL maintainer="Sooo"

# 时区
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y tzdata && \
    ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime && \
    echo ${TZ} > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

# TensorRT
COPY TensorRT.tar.gz tensorRT.tar.gz
RUN tar -xzvf tensorRT.tar.gz -C /opt && \
    rm tensorRT.tar.gz
ENV LD_LIBRARY_PATH=/opt/TensorRT/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

#  基础软件包
RUN apt update && \
    apt install -y g++ cmake doxygen && \
    apt install -y libceres-dev libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

#  海康相机库
COPY MVS.tar.gz MVS.tar.gz
RUN tar -xzvf MVS.tar.gz -C /opt && \
    rm MVS.tar.gz
ENV LD_LIBRARY_PATH=/opt/MVS/lib/64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

#  git安装
RUN apt update && \
    apt install -y git curl && \
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt install git-lfs && \
    git lfs install && \
    rm -rf /var/lib/apt/lists/*

#  gitlab测试环境
RUN curl -L "https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh" | bash && \
    apt install -y gitlab-runner && \
    rm -rf /var/lib/apt/lists/*
CMD ["sh","-c","/bin/gitlab-runner run"]

ENV https_proxy "127.0.0.1:7890"
