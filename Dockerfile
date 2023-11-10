FROM nvidia/cuda:12.2.2-devel-ubuntu20.04

# 设置时区并避免交互式对话框
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# 常规的Ubuntu工具
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    tzdata

# 创建sam用户
#RUN useradd -ms /bin/bash sam
#ENV USERNAME sam

# 安装ROS 1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - && \
    apt-get update && apt-get install -y ros-noetic-desktop

# 安装ROS 2 Humble
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0x23EFEFE93C4CFFFE' | apt-key add - && \
    apt-get update && apt-get install -y \
    ros-galactic-desktop \
    ros-galactic-ros1-bridge \
    ros-galactic-rosbag2-bag-v2-plugins

# Install zsh and git for oh-my-zsh
RUN apt-get install -y zsh git

# Create a .zshrc for sam to avoid oh-my-zsh prompts
#USER sam
RUN touch ~/.zshrc

# Switch to zsh and install oh-my-zsh
SHELL ["zsh", "-c"]
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    echo "plugins=(zsh-autosuggestions)" >> ~/.zshrc
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/galactic/setup.zsh" >> ~/.zshrc

USER root
RUN apt-get install -y gedit python3-colcon-common-extensions wget ros-galactic-sensor-msgs-py && \
    rm -rf /var/lib/apt/lists/*
    
WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/Mikehuntisbald/ros2_kitti_publishers
RUN /bin/zsh -c '. /opt/ros/galactic/setup.zsh; cd .. && colcon build'

# 安装Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
RUN chmod +x miniconda.sh
RUN ./miniconda.sh -b -p /opt/conda
RUN rm miniconda.sh
#ros2 run ros2_kitti_publishers kitti_publishers

# 将conda命令添加到环境变量中，以便可以直接使用
ENV PATH="/opt/conda/bin:${PATH}"

# 复制你的environment.yml到Docker容器中
COPY environment.yml /tmp/environment.yml

# 使用environment.yml文件创建conda环境
RUN conda env create -f /tmp/environment.yml

# 清除不必要的缓存和临时文件
RUN conda clean --all --yes

# 设置要激活的conda环境
ENV CONDA_DEFAULT_ENV=open_pcd
ENV CONDA_PREFIX=/opt/conda/envs/$CONDA_DEFAULT_ENV
ENV PATH=$CONDA_PREFIX/bin:$PATH

WORKDIR /home/catkin_ws/data/2011_09_26
WORKDIR /home/catkin_ws
RUN source install/setup.zsh
RUN echo "source /opt/conda/etc/profile.d/conda.sh" >> /root/.zshrc

#RUN /bin/zsh -c "conda activate open_pcd"

WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/Mikehuntisbald/OpenPCDet
WORKDIR /home/catkin_ws/src/OpenPCDet
SHELL ["conda", "run", "-n", "open_pcd", "/bin/zsh", "-c"]
RUN python setup.py develop

# 设置入口点
ENTRYPOINT ["zsh"]

