# given by command,
#   sudo docker pull stereolabs/zed:4.0-tools-devel-jetson-jp6.0
FROM stereolabs/zed:4.0-tools-devel-jetson-jp6.0

ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra

# install ROS
RUN locale  # check for UTF-8
RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale  # verify settings
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe
RUN sudo apt update && sudo apt install curl -y

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update
RUN sudo apt upgrade -y

# install ROS ==> install Humble essentials
RUN sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    python3-argcomplete \
    ros-dev-tools \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-xacro \
    ros-humble-diagnostic-updater \
    ros-humble-robot-localization \
    ros-humble-std-msgs \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs

RUN echo "Hello, World"

# install starter packs
RUN apt update
RUN apt install -y \
                    build-essential \
                    cmake \
                    git \
                    libbullet-dev \
                    python3-colcon-common-extensions \
                    python3-flake8 \
                    python3-pip \
                    python3-pytest-cov \
                    python3-rosdep \
                    python3-setuptools \
                    python3-vcstool \
                    wget

# install python packages
RUN python3 -m pip install -U \
                    argcomplete \
                    flake8-blind-except \
                    flake8-builtins \
                    flake8-class-newline \
                    flake8-comprehensions \
                    flake8-deprecated \
                    flake8-docstrings \
                    flake8-import-order \
                    flake8-quotes \
                    pytest-repeat \
                    pytest-rerunfailures \
                    pytest

# install dependencies
RUN sudo apt install --no-install-recommends \
                    libasio-dev \
                    libtinyxml2-dev \
                    libcunit1-dev

# set ROS bash settings
RUN echo "source /opt/ros/humble/setup.bash" \
        "\nsource ~/kros_ws/install/local_setup.bash" \
        "\nsource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" \
        "\nsource /usr/share/vcstool-completion/vcs.bash" \
        "\nsource /usr/share/colcon_cd/function/colcon_cd.sh" \
        "\nexport _colcon_cd_root=~/kros_ws" \
        "\nexport RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
        "\nexport RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'" \
        "\nexport RCUTILS_COLORIZED_OUTPUT=1" \
        "\nexport RCUTILS_LOGGING_USE_STDOUT=0" \
        "\nexport RCUTILS_LOGGING_BUFFERED_STREAM=1" \
        "\nalias cw='cd ~/kros_ws'" \
        "\nalias cs='cd ~/kros_ws/src'" \
        "\nalias ccd='colcon_cd'" \
        "\nalias cb='cd ~/robot_ws && colcon build --symlink-install'" \
        "\nalias cbs='colcon build --symlink-install'" \
        "\nalias cbp='colcon build --symlink-install --packages-select'" \
        "\nalias cbu='colcon build --symlink-install --packages-up-to'" \
        "\nalias ct='colcon test'" \
        "\nalias ctp='colcon test --packages-select'" \
        "\nalias ctr='colcon test-result'" \
        "\nalias rt='ros2 topic list'" \
        "\nalias re='ros2 topic echo'" \
        "\nalias rn='ros2 node list'" >> /root/.bashrc

# install nautilus, vim
RUN sudo apt install nautilus -y
RUN sudo apt install vim -y

# make workspace
RUN mkdir -p /root/kros_ws/src
WORKDIR /root/kros_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install 
WORKDIR /root/kros_ws/install
RUN chmod +x ./local_setup.sh


# download px4_ros_com_ros2
RUN sudo apt install openjdk-11-jdk -y
WORKDIR /root
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen
WORKDIR /root/Fast-RTPS-Gen/gradle/wrapper
RUN sed -i '3s|distributionUrl=.*|distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip|' /root/Fast-RTPS-Gen/gradle/wrapper/gradle-wrapper.properties
WORKDIR /root/Fast-RTPS-Gen
RUN ./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install

RUN sudo apt install ros-humble-eigen3-cmake-module -y

RUN sudo apt install python3-testresources -y
RUN sudo pip3 install -U empy pyros-genmsg setuptools


# build px4_ros_com_ros2
WORKDIR /root/kros_ws/src
RUN git clone -b release/1.13 https://github.com/PX4/px4_ros_com.git
RUN git clone -b release/1.13 https://github.com/PX4/px4_msgs.git
WORKDIR /root/kros_ws/src/px4_ros_com/scripts
RUN bash -c './build_ros2_workspace.bash'



# install zed wrapper
RUN sudo apt remove -y ros-humble-image-transport-plugins ros-humble-compressed-depth-image-transport ros-humble-compressed-image-transport
WORKDIR /root/kros_ws/src
RUN git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch # clone the "v3.0.0" branch of the "image_common" repository
WORKDIR /root/kros_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

RUN apt remove -y ros-humble-image-transport-plugins ros-humble-compressed-depth-image-transport ros-humble-compressed-image-transport

RUN . /opt/ros/humble/setup.sh

WORKDIR /root/kros_ws/src
RUN git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
WORKDIR /root/kros_ws
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths /root/kros_ws/src --ignore-src -r -y

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-up-to zed_wrapper --allow-overriding image_transport --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" --parallel-workers $(nproc)

# build zed-rgb convert
WORKDIR /root/kros_ws/src
RUN git clone https://github.com/joony414/custom_zed_rgb_convert 
WORKDIR /root/kros_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-up-to zed_rgb_convert --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" --parallel-workers $(nproc)


# ==============================================================================
# custom_yolov8_ros 삭제 후 YOLO11(ultralytics) 설치만 추가(추후 수정 필요)

# install yolo11 (Ultralytics YOLO11)
RUN python3 -m pip install -U pip
RUN python3 -m pip install "ultralytics[export]"

# JetPack 6 (Python 3.10) - install Jetson-compatible PyTorch/Torchvision wheels
# (NVIDIA forum에서 안내된 실동작 커맨드 기반)  :contentReference[oaicite:4]{index=4}
RUN python3 -m pip uninstall -y torch torchvision torchaudio || true
RUN python3 -m pip install \
    https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl

RUN wget https://nvidia.box.com/shared/static/u0ziu01c0kyji4zz3gxam79181nebylf.whl -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl && \
    python3 -m pip install torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl && \
    rm -f torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

RUN wget https://nvidia.box.com/shared/static/9si945yrzesspmg9up4ys380lqxjylc3.whl -O torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl && \
    python3 -m pip install torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl && \
    rm -f torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl

# onnxruntime-gpu (JetPack 6 + Python3.10) - Ultralytics Jetson guide  :contentReference[oaicite:5]{index=5}
RUN python3 -m pip install \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl


# custom usb_cam (gopro)
RUN sudo apt install ffmpeg v4l2loopback-dkms curl vlc -y
RUN sudo su -c "bash <(wget -qO- https://cutt.ly/PjNkrzq)" root
WORKDIR /root/kros_ws/src
RUN git clone https://github.com/joony414/custom_usb_cam
WORKDIR /root/kros_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-up-to usb_cam
