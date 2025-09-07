ARG ROS_DISTRO="osrf/ros:humble-desktop"
FROM $ROS_DISTRO

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG WEBOTS=false

# Install base packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    python3-pip \
    wget \
    sudo \
    python3-cv-bridge \
    python3-opencv \
    v4l-utils \
    # Picamera2 + libcamera dependencies
    libcamera0 \
    libcamera-tools \
    libcamera-dev \
    libcap-dev \
    python3-prctl

# Conditionally install Webots based on WEBOTS argument
ARG WEBOTS_VERSION=2025a
RUN if [ "$WEBOTS" = "true" ]; then \
        apt-get update && apt-get install -y \
        libgl1 \
        libqt5gui5 \
        libjpeg-dev \
        libpulse0 \
        ros-humble-webots-ros2 && \
        WEBOTS_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"; \
        wget ${WEBOTS_URL} -O /tmp/webots.deb && \
        apt-get install -y /tmp/webots.deb && \
        rm /tmp/webots.deb; \
    fi

# Create user and group with proper handling of existing users and groups
RUN if getent group $USER_GID > /dev/null 2>&1; then \
        # Group exists, check if it's the right name
        existing_group=$(getent group $USER_GID | cut -d: -f1); \
        if [ "$existing_group" != "$USERNAME" ]; then \
            groupmod -n $USERNAME $existing_group; \
        fi; \
    else \
        # Group doesn't exist, create it
        groupadd -g $USER_GID $USERNAME; \
    fi \
    && if getent passwd $USER_UID > /dev/null 2>&1; then \
        # User exists, modify it to match our requirements
        existing_user=$(getent passwd $USER_UID | cut -d: -f1); \
        if [ "$existing_user" != "$USERNAME" ]; then \
            usermod -l $USERNAME -d /home/$USERNAME -m $existing_user; \
        fi; \
        usermod -g $USERNAME $USERNAME; \
    else \
        # User doesn't exist, create it
        useradd -u $USER_UID -g $USER_GID -m $USERNAME; \
    fi \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME/camerabot
ENV HOME=/home/$USERNAME
ENV USER=$USERNAME

# Copy requirements.txt and install Python dependencies
COPY src/robot/requirements.txt /home/$USERNAME/requirements.txt

# Source ROS 2 and install Python dependencies
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc \
    && echo "cd /home/$USERNAME/camerabot" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    && pip3 install -r /home/$USERNAME/requirements.txt \
    # Install Picamera2 in the image so camera works in container
    && pip3 install --no-cache-dir picamera2

# Set default command
CMD ["bash"] 