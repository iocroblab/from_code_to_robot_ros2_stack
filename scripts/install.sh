#!/usr/bin/env bash
set -euo pipefail

# install.sh
#
# Modes:
#   fulldocker  -> install Docker Engine + build image + create named container (stopped)
#   docker      -> build image + create named container (stopped)
#   fullnative  -> install ROS 2 Jazzy + required apt packages + setup workspace (clone/vcs/rosdep/build)
#   native      -> setup workspace only (assumes ROS 2 Jazzy + tools already installed)
#
# Notes:
# - Native setup mirrors the ROS-related packages from the Dockerfile (ROS Jazzy + ros-gz + ros2_control + DDS).
# - We do NOT install the container-only VNC/noVNC/sshd stack on native.
# - We create a named Docker container but keep it STOPPED for MATLAB to attach later.

MODE="${1:-docker}"

# ---- Shared config ----
ROS_DISTRO="jazzy"
IMAGE="from-code-to-robot-tutorial-docker"
CNAME="FCTR-container"
TURTLEBOT3_MODEL_DEFAULT="waffle"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/../docker"

# Native workspace location
WS_DIR="${WS_DIR:-$HOME/fctr_ws}"
STACK_REPO_URL="https://github.com/iocroblab/from_code_to_robot_ros2_stack.git"
STACK_REPO_BRANCH="jazzy"

# ------------------------
# Helper functions
# ------------------------
log() { echo -e "\n==> $*\n"; }

ubuntu_codename() {
  . /etc/os-release
  echo "${VERSION_CODENAME}"
}

ensure_command() {
  local cmd="$1"
  if ! command -v "$cmd" &>/dev/null; then
    echo "ERROR: Required command not found: $cmd"
    exit 1
  fi
}

# ------------------------
# Docker tasks
# ------------------------
install_docker_engine() {
  log "Installing Docker Engine (not Docker Desktop)..."

  sudo apt update
  sudo apt install -y ca-certificates curl

  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  local CODENAME
  CODENAME="$(ubuntu_codename)"

  sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${CODENAME}
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

  sudo apt update
  sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

  log "Configuring Docker permissions for current user..."
  sudo groupadd -f docker
  sudo usermod -aG docker "$USER"

  echo "NOTE: Docker group membership change needs logout/login to take effect."
  echo "This script will keep using sudo docker to be safe."
}

ensure_docker_present() {
  ensure_command docker
}

build_image() {
  log "Building Docker image: $IMAGE"
  sudo docker build -t "$IMAGE" "$DOCKER_DIR"
}

create_or_refresh_named_container() {
  log "Creating (or recreating) named container (stopped): $CNAME"

  # Remove old container if present (avoid name conflict / stale config)
  sudo docker rm -f "$CNAME" >/dev/null 2>&1 || true

  # X access (native Linux). On WSL this often needs different X11 setup.
  xhost +local:docker >/dev/null 2>&1 || true

  # Create container but DO NOT start it.
  # Keepalive command makes `docker start $CNAME` keep it running even without attach.
  sudo docker create \
    --name "$CNAME" \
    --env="DISPLAY=${DISPLAY:-:0}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/snd \
    --net=host \
    "$IMAGE" \
    bash -lc "trap : TERM INT; sleep infinity & wait" >/dev/null

  echo "OK: container created and STOPPED: $CNAME"
}

smoke_test_image() {
  log "Smoke test (non-interactive; does not affect $CNAME)..."
  sudo docker run --rm "$IMAGE" bash -lc 'echo "OK: image runs"'
}

print_next_steps_docker() {
  echo
  echo "Done with Docker and workspace setup."
  echo "Named container (stopped): $CNAME"
  echo "Image: $IMAGE"
  echo
  echo "Test your workspace from MATLAB via StartTutorialApplication('Rviz')"
  echo "Send a sample configuration by running JointStatesToRviz([0,0,0,0,0,0]) in MATLAB."
  echo
}

# ------------------------
# Native tasks
# ------------------------
install_ros2_jazzy_and_deps() {
  log "Installing ROS 2 ${ROS_DISTRO} + required apt packages (native)..."

  # Locale (mirrors Dockerfile intent)
  sudo apt update
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

  # Universe + tooling
  sudo apt install -y software-properties-common
  sudo add-apt-repository -y universe

  # Key + repo
  sudo apt update
  sudo apt install -y curl gnupg2 lsb-release wget

  sudo install -d -m 0755 /usr/share/keyrings
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  # Ensure no conflicting old list
  sudo rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list

  local CODENAME
  CODENAME="$(ubuntu_codename)"

  # ROS 2 apt repo
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  sudo apt update
  sudo apt upgrade -y

  # Base ROS desktop (matches Dockerfile target)
  sudo apt install -y "ros-${ROS_DISTRO}-desktop"

  # Dev tools + python tooling (mirrors Dockerfile list)
  sudo apt install -y \
    python3-pip \
    python3-flake8-docstrings \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    "ros-${ROS_DISTRO}-ros-dev-tools" \
    python3-colcon-common-extensions

  # DDS (mirrors Dockerfile)
  sudo apt install -y \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-rmw-fastrtps-dynamic-cpp"

  # Gazebo bridge + control stack (mirrors Dockerfile)
  sudo apt install -y \
    "ros-${ROS_DISTRO}-ros-gz" \
    "ros-${ROS_DISTRO}-gz-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-controller-manager" \
    "ros-${ROS_DISTRO}-turtlebot3-msgs"

  # Workspace tooling used later
  sudo apt install -y git python3-vcstool python3-rosdep

  # rosdep init/update (idempotent-ish)
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update

  log "ROS 2 ${ROS_DISTRO} native installation complete"
}

setup_workspace_native() {
  log "Setting up native workspace: $WS_DIR"

  # shellcheck disable=SC1091
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # ROS setup scripts are not always safe with `set -u`
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    else
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found."
    echo "If you intended to install ROS first, run: bash install.sh fullnative"
    exit 1
    fi

  mkdir -p "$WS_DIR/src/modified-repositories"
  cd "$WS_DIR"

  # Clone stack repo (branch jazzy)
  if [ ! -d "$WS_DIR/src/from_code_to_robot_ros2_stack" ]; then
    git clone -b "$STACK_REPO_BRANCH" --depth 1 "$STACK_REPO_URL" "$WS_DIR/src/from_code_to_robot_ros2_stack"
  else
    log "Stack repo already exists, updating..."
    (cd "$WS_DIR/src/from_code_to_robot_ros2_stack" && git fetch --depth 1 origin "$STACK_REPO_BRANCH" && git checkout "$STACK_REPO_BRANCH" && git pull --ff-only) || true
  fi

  # Import repos into modified-repositories (mirrors Dockerfile)
  vcs import "$WS_DIR/src/modified-repositories" < "$WS_DIR/src/from_code_to_robot_ros2_stack/stack.repos"

  # Optional repos file (mirrors Dockerfile logic)
  OPTIONAL_REPOS_FILE="$WS_DIR/src/modified-repositories/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos"
  if [ -f "$OPTIONAL_REPOS_FILE" ]; then
    vcs import "$WS_DIR/src" --skip-existing --input "$OPTIONAL_REPOS_FILE"
  else
    echo "Optional repos file not found, skipping: $OPTIONAL_REPOS_FILE"
  fi

  # rosdep + build
  rosdep update
  rosdep install --ignore-src --from-paths src -y

  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  log "Workspace build complete"
}

print_next_steps_native() {
  log "Native setup complete"
  echo "Workspace: $WS_DIR"
  echo
  echo "Test your workspace from MATLAB via StartTutorialApplication('Rviz','docker',false)"
  echo "Send a sample configuration by running JointStatesToRviz([0,0,0,0,0,0]) in MATLAB."
  echo
}

# ------------------------
# Mode dispatch
# ------------------------
case "$MODE" in
  fulldocker)
    install_docker_engine
    ensure_docker_present
    build_image
    create_or_refresh_named_container
    smoke_test_image
    print_next_steps_docker
    ;;
  docker)
    ensure_docker_present
    build_image
    create_or_refresh_named_container
    smoke_test_image
    print_next_steps_docker
    ;;
  fullnative)
    install_ros2_jazzy_and_deps
    setup_workspace_native
    print_next_steps_native
    ;;
  native)
    # Minimal checks for workspace-only setup
    ensure_command git
    ensure_command vcs
    ensure_command rosdep
    ensure_command colcon
    setup_workspace_native
    print_next_steps_native
    ;;
  *)
    echo "Unknown mode: $MODE"
    echo "Usage: $0 [docker|fulldocker|native|fullnative]"
    exit 1
    ;;
esac
