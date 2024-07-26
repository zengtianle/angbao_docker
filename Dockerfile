FROM arm64v8/ros:kinetic-perception

# 设置全局代理服务器
#ENV all_proxy=http://nvidia:nvidia@10.21.61.30:808
ENV HTTP_PROXY=http://nvidia:nvidia@10.21.61.30:808
ENV HTTPS_PROXY=http://nvidia:nvidia@10.21.61.30:808

# 安装常见应用
RUN apt-get update &&\
    apt-get install -y curl wget vim git zip unzip openssh-client inetutils-ping python3-pip

# 设置工作目录
WORKDIR /root/catkin_ws

# 创建 ROS 工作空间
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"

# 将功能包复制到工作空间的 src 目录
COPY /home/nvidia/work/autoslam_gm  /root/catkin_ws/src/autoslam_gm

# 编译工作空间
RUN apt-get install -y python-serial libserial-dev libsdl1.2-dev libbullet-dev libsdl-image1.2-dev ros-kinetic-desktop-full 
RUN apt-get install -y ros-kinetic-serial ros-kinetic-map-msgs ros-kinetic-voxel-grid
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash "
#&& cd /root/catkin_ws && catkin_make"

