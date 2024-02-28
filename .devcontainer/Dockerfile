ARG CUDA_VERSION=12.2.2
ARG CUDNN_VERSION=8
ARG OS_VERSION=20.04

# 拉取基础镜像
FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${OS_VERSION}
LABEL maintainer="Sooo"

WORKDIR /Workspace

# 时区
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y tzdata && \
    ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime && \
    echo ${TZ} > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
    apt install -y wget && \
    apt install -y curl && \
    && apt install python3-yaml python3-pip -y  \
    && pip3 install distro \
    rm -rf /var/lib/apt/lists/*

# ROS Noetic
COPY fish_install.yaml ./fish_install.yaml
RUN wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt-get clean && apt autoclean

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
    apt install -y git && \
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt install git-lfs && \
    git lfs install && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && \
   apt install -y git vim zsh && \
   rm -rf /var/lib/apt/lists/*

RUN wget --no-check-certificate https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh && \
      git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
   git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
   chsh -s $(which zsh) && \
   rm -rf /var/lib/apt/lists/*
    
COPY zshrc /root/.zshrc
RUN unset https_proxy
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc
RUN echo PATH="/root/miniconda3/bin":$PATH >> ~/.zshrc

CMD ["/bin/zsh"]


