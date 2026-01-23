##############################################################################
# Base Image: ros2-humble

##############################################################################

ARG ROS_DISTRO=humble
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base-jammy

# Build arguments for platform detection
ARG TARGETPLATFORM
ARG BUILDPLATFORM
ARG TARGETARCH


##############################################################################
# Arguments
##############################################################################
ARG USERNAME=main
ARG USER_UID=1000
ARG USER_GID=1000
ARG SHELL=bash
ARG ENABLE_VNC=false

##############################################################################
# Display build information
##############################################################################

RUN echo "Building for platform: ${TARGETPLATFORM:-unknown}" && \
    echo "Architecture: ${TARGETARCH:-unknown}" && \
    uname -m

##############################################################################
#LABEL
##############################################################################

ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}
ENV TERM=xterm-256color
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}

##############################################################################
# Cache Setting
##############################################################################

# keep downloaded packages for caching purposes
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# upgrade packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
# Install packages
##############################################################################

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    sudo \
    vim \
    curl \
    wget \
    tmux \
    htop \
    git \
    git-extras \
    gnupg2 \
    net-tools \
    locales \
    build-essential \
    cmake \
    unzip \
    python3-pip \
    bash-completion \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for ROS and Nav2
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    libsdl-image1.2-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libyaml-cpp-dev \
    ros2-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros2-${ROS_DISTRO}-navigation2 \
    ros2-${ROS_DISTRO}-foxglove-bridge \ 
    ros2-${ROS_DISTRO}-ament-cmake \
    ros2-${ROS_DISTRO}-rqt-plot \
    ros2-${ROS_DISTRO}-rqt-tf-tree \
    ros2-${ROS_DISTRO}-nav2-msgs \
    ros2-${ROS_DISTRO}-nav2-util \
    # Groot
    libfuse2 fuse3 \
    ros2-${ROS_DISTRO}-behaviortree-cpp \
    ros2-${ROS_DISTRO}-generate-parameter-library \
    ros2-${ROS_DISTRO}-teleop-twist-keyboard \
    ros2-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
# Set Language
##############################################################################

RUN echo "en_US.UTF-8 UTF-8" > /etc/local.gen && locale-gen en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
##############################################################################
# Set timezone (Taipei)
##############################################################################

RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

##############################################################################
# Add new user (Passwordless with sudo)
##############################################################################

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid ${USERGID} ${USERNAME} \
    && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
    # passwordless
    && apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
# Set Default User & Working Directory & Default .bashrc
##############################################################################

USER ${USERNAME}
WORKDIR /home/${USERNAME}
COPY ./settings/.bashrc /home/${USERNAME}/.bashrc    

##############################################################################
# VNC Installation
##############################################################################

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    if [ "$ENABLE_VNC" = "true" ]; then \
        echo "Installing VNC support..."; \
        /usr/local/bin/install_vnc.sh; \
    else \
        echo "Skipping VNC installation"; \
    fi

USER root
RUN echo 'user:user' | chpasswd

##############################################################################
# Clean apt-cache
##############################################################################

RUN sudo apt autoremove -y && \
    sudo apt clean -y 

##############################################################################
# Environment Setup and entrypoint
##############################################################################
USER ${USERNAME}
WORKDIR /home/${USERNAME}

EXPOSE 5901

# Set up the default command
SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]