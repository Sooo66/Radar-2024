ARG CUDA_VERSION=12.2.2
ARG CUDNN_VERSION=8
ARG OS_VERSION=20.04

# 拉取基础镜像
FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${OS_VERSION}
LABEL maintainer="Sooo"

WORKDIR /root

# 时区
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y tzdata && \
    ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime && \
    echo ${TZ} > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

#  请将此处修改为梯子的代理端口，否则无法连接到Github
# ENV https_proxy "127.0.0.1:7890"

RUN apt update && \
    apt install -y wget && \
    apt install -y curl && \
    rm -rf /var/lib/apt/lists/*

# ROS Noetic
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
# RUN apt update && \
#     apt install -y -f ros-noetic-desktop-full
# RUN apt install -y -f python3-rosdep
# RUN rosdep init \
#     && rosdep fix-permissions \
#     && rosdep update
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# RUN apt update \
#  && apt install -y --no-install-recommends ros-noetic-desktop-full
# RUN apt install -y --no-install-recommends python3-rosdep
# RUN rosdep init \
#  && rosdep fix-permissions \
#  && rosdep update
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# RUN wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && \
#     chmod +x ./ros_install_noetic.sh && \
#     ./ros_install_noetic.sh

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

# Miniconda 
RUN wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && mkdir /root/.conda \
    && bash Miniconda3-latest-Linux-x86_64.sh -b \
    && rm -f Miniconda3-latest-Linux-x86_64.sh \
    && echo PATH="/root/miniconda3/bin":$PATH >> ~/.bashrc \
    && exec bash \
    && conda --version \
    && conda create -n srm python=3.11 \
    && conda activate srm \
    && pip3 install flask

RUN apt update && \
   apt install -y git wget vim zsh && \
   rm -rf /var/lib/apt/lists/*

RUN wget --no-check-certificate https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh && \
      git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
   git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
   chsh -s $(which zsh) && \
   rm -rf /var/lib/apt/lists/*

COPY fish_install.yaml fish_install.yaml
RUN apt update \ 
    && apt install wget python3-yaml -y  \
    && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    # 进行最后的清理
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt-get clean && apt autoclean 
    
COPY zshrc /root/.zshrc
RUN unset https_proxy
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc
RUN echo PATH="/root/miniconda3/bin":$PATH >> ~/zshrc

CMD ["/bin/zsh"]
CMD ["conda", "activate", "srm"]




