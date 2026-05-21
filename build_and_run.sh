#!/usr/bin/bash
set -euo pipefail

ros_distro=$1
container_name=$2
image_name=$3
webots=${4:-false}  # Default to false (GUI mode)
x11=${5:-false}  # Default to false (GUI mode)

export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$USER
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# GNOME/Wayland often sets XAUTHORITY to /run/user/$UID/.mutter-Xwaylandauth.* which changes
# between logins; bind-mounting that path into the container breaks later `docker start`.
STABLE_XAUTH="/tmp/camerabot-docker-${USER_UID}.xauth"
XAUTH_IN_CONTAINER="/tmp/camerabot-xauthority"

sync_xauthority_copy() {
  local xauth_path=${XAUTHORITY:-$HOME/.Xauthority}
  if [ ! -f "$xauth_path" ]; then
    return 0
  fi
  cp -f -- "$xauth_path" "$STABLE_XAUTH"
  chmod 600 "$STABLE_XAUTH"
}

# If container exists, exec into it (start if needed)
container_status=$(docker inspect -f '{{.State.Status}}' "$container_name" 2>/dev/null || true)
if [ "${container_status:-}" = "running" ]; then
  echo "📦 Container '$container_name' is already running. Executing shell..."
  exec docker exec -it "$container_name" bash
elif [ "${container_status:-}" = "exited" ] || [ "${container_status:-}" = "created" ] || [ "${container_status:-}" = "paused" ]; then
  echo "📦 Container '$container_name' exists but is not running. Starting and executing shell..."
  if [ "$x11" = "true" ]; then
    sync_xauthority_copy || true
  fi
  if ! docker start "$container_name"; then
    echo ""
    echo "❌ docker start failed. If mounts mention .mutter-Xwaylandauth or /run/user/, recreate:"
    echo "   docker rm -f \"$container_name\""
    echo "   Then run this script again (X11 cookie is copied to $STABLE_XAUTH for a stable bind mount)."
    echo ""
    exit 1
  fi
  exec docker exec -it "$container_name" bash
fi

# Stop and remove existing container
echo "🛑 Stopping existing container..."
docker stop $container_name 2>/dev/null || true
docker rm $container_name 2>/dev/null || true

# Resolve host group IDs for common device groups so the in-container user
# gets proper access to /dev entries when /dev is bind-mounted.
get_gid() { getent group "$1" | cut -d: -f3 || true; }

VIDEO_GID=$(get_gid video)
RENDER_GID=$(get_gid render)
DIALOUT_GID=$(get_gid dialout)
INPUT_GID=$(get_gid input)
GPIO_GID=$(get_gid gpio)
I2C_GID=$(get_gid i2c)
SPI_GID=$(get_gid spi)
PLUGDEV_GID=$(get_gid plugdev)

# Build supplemental group args, using numeric GIDs so they match host /dev nodes
GROUP_ARGS=""
for gid in "$VIDEO_GID" "$RENDER_GID" "$DIALOUT_GID" "$INPUT_GID" "$GPIO_GID" "$I2C_GID" "$SPI_GID" "$PLUGDEV_GID"; do
  if [ -n "$gid" ]; then
    GROUP_ARGS="$GROUP_ARGS --group-add $gid"
  fi
done

#  3. Create Docker network for ROS2 containers
# echo "🌐 Creating ROS2 network..."
# docker network create camerabot_network 2>/dev/null || echo "Network already exists"

#  4. Build and run Docker container 
echo "🔨 Building ROS 2 Docker image for" $ros_distro " webots:" $webots " x11:" $x11 "..."
# Build the Docker image with platform with specification and build args
if [ "$(docker images -q $image_name 2> /dev/null)" == "" ]; then
docker build \
  --build-arg ROS_DISTRO=$ros_distro \
  --build-arg WEBOTS=$webots \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t $image_name .
fi

# Set up X11 forwarding only if not in headless mode
if [ "$x11" == "true" ]; then
  echo "🖥️ Setting up X11 forwarding for GUI mode..."
  xhost +local:docker
  
  XAUTH_PATH=${XAUTHORITY:-$HOME/.Xauthority}
  if [ -f "$XAUTH_PATH" ]; then
    echo "🔐 XAUTHORITY from $XAUTH_PATH → bind mount $STABLE_XAUTH (stable path for Docker)"
    sync_xauthority_copy
    XAUTH_VOLUME="--volume $STABLE_XAUTH:$XAUTH_IN_CONTAINER:ro"
    XAUTH_ENV="--env XAUTHORITY=$XAUTH_IN_CONTAINER"
  else
    echo "⚠️  XAUTHORITY not found, X11 may not work properly"
    XAUTH_VOLUME=""
    XAUTH_ENV=""
  fi
  
  X11_ARGS="--env QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  --env DISPLAY=$DISPLAY \
  --env _XEVENT_TRACE=1 \
  --env QT_DEBUG_PLUGINS=1 \
  --env QT_QPA_PLATFORM=xcb \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  $XAUTH_VOLUME \
  $XAUTH_ENV"
else
  echo "🚫 Running in headless mode (no GUI support)"
  X11_ARGS=""
fi

echo "🚀 Starting ROS 2 container..."

# Run the container with network configuration and camera support
docker run -it \
  --name $container_name \
  --tty \
  --interactive \
  --network host \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  $X11_ARGS \
  --volume .:/home/$USERNAME/camerabot:rw \
  --volume /dev:/dev \
  --volume /run/udev:/run/udev:ro \
  --device /dev/vchiq:/dev/vchiq \
  --device /dev/video0:/dev/video0 \
  --device /dev/video1:/dev/video1 \
  --device /dev/media0:/dev/media0 \
  --device /dev/media1:/dev/media1 \
  $GROUP_ARGS \
  --privileged \
  $image_name \
  bash -c "echo 📷 Enabling camera access for Raspberry Pi... && bash rpicam2ubuntu.sh &&\
           echo ✅  Done! Next steps: &&\
           echo build packages: colcon build --symlink-install &&\
           echo every time source environment: . e &&\
           bash"