# Installing and Setting Up Franka Research 3 ROS2

This guide covers the installation of necessary dependencies and the setup of the `franka_ros2` package for controlling Franka Emika robots using ROS 2 Humble.

## 1. Dependencies

### 1.1. libFranka

Before proceeding, ensure you have the following base dependencies installed:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev dpkg
```

#### 1.1.1. Pinocchio (for libFranka 0.14.0 or later)

libFranka 0.14.0 and newer versions require Pinocchio for kinematics and dynamics calculations. Install the necessary dependencies for Pinocchio:

```bash
sudo apt-get install -y liburdfdom-headers-dev libconsole-bridge-dev libtinyxml2-dev
```

Install additional required packages:

```bash
sudo apt install -qqy lsb-release curl
```

Register the authentication certificate of robotpkg:

```bash
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
```

Add robotpkg as a source repository to apt:

```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

Update apt to fetch the package descriptions:

```bash
sudo apt update
```

Install Pinocchio and its dependencies (use `robotpkg-py310-pinocchio` for ROS 2 and Ubuntu 22.04):

```bash
sudo apt install -qqy robotpkg-py310-pinocchio
```

#### 1.1.2. Setting Environment Variables

To ensure that your system can locate the installed libraries, add the following lines to your `$HOME/.bashrc` file. This will make the configuration persistent across terminal sessions. Otherwise you would need to run them in each terminal:

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

**Important:**  After modifying your `.bashrc`, either source it (`source ~/.bashrc`) or open a new terminal to apply the changes.

### 1.2. Building libFranka

Before building from source, remove any existing installations of libFranka to prevent conflicts:

```bash
sudo apt remove "*libfranka*"
```

#### 1.2.1. Clone the Repository

Clone the `libfranka` repository and choose the desired version using Git tags:

```bash
# Clone the repository
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka

# List available tags
git tag -l

# Checkout a specific tag (e.g., 0.14.1)
git checkout 0.14.1

# Update submodules
git submodule update
```

#### 1.2.2. Build and Install

Create a build directory, configure the project with CMake, and build:

```bash
# Create a build directory and navigate to it
mkdir build
cd build

# You may encounter problems during the cmake configure process
# without first setting:
export CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake  # for pinocchio, etc...
# or as a parameter to cmake explicitely.

# Configure the project and build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF [-DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake] ..
cmake --build .
```

#### 1.2.3. Installing as a Debian Package (Optional but Recommended)

Building a Debian package simplifies installation and management. To create a Debian package, run the following in the `build` directory:

```bash
cpack -G DEB
```

This creates a `.deb` package (e.g., `libfranka-<version>-<architecture>.deb`). Install it using:

```bash
sudo dpkg -i libfranka*.deb
```

**Benefits of using a Debian package:**

-   Easier installation and uninstallation.
-   Better integration with system package managers.
-   Simplified updates and dependency management.

## 2. Franka ROS 2 (`franka_ros2`)

Now that the dependencies are resolved, we can proceed with setting up the `franka_ros2` package.

### 2.1. Setup

#### 2.1.1. Install from Source

1. **Install ROS 2 Packages:**

    Install the necessary ROS 2 packages using `apt`:
    ```bash
    sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-angles \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-generate-parameter-library \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-pinocchio \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-hardware-interface \
    ros-humble-ros-gz \
    python3-colcon-common-extensions
    ```

2. **Create a ROS 2 Workspace:**

    ```bash
    mkdir -p ~/franka_ros2_ws/src
    ```

3. **Clone Repositories and Build:**

    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/franka_ros2_ws
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
    git clone https://github.com/frankaemika/franka_description.git src/franka_description
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.sh
    ```

### 2.2. Test the Setup

To verify your setup without a physical robot, launch the simulation with dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

This command starts RViz and MoveIt, allowing you to interact with a simulated Franka Emika robot. If the simulation runs without errors, your setup is likely correct.
