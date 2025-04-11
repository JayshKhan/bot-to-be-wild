# Getting Started with Franka Emika Robots and ROS: Setup, MoveIt!, Pick-and-Place, and Optional Xavier NX Integration

## 1. Introduction

This report provides a comprehensive, step-by-step guide for setting up and operating Franka Emika robotic arms (Panda and FR3 models) using the Robot Operating System (ROS). It covers the essential procedures from initial hardware connection and network configuration to software installation, basic control verification, integration with the MoveIt! motion planning framework, and implementation of a fundamental pick-and-place task. Additionally, an optional section details the integration of an NVIDIA Jetson Xavier NX for executing Reinforcement Learning (RL) policies, outlining the necessary setup and communication architecture.

The primary focus is on ROS 1 Noetic Ninjemys due to its established support within the franka_ros ecosystem, but relevant information and distinctions for ROS 2 (Humble Hawksbill/Foxy Fitzroy) are included where applicable.

This guide assumes the Franka Emika robot arm and controller are physically installed and connected as per the manufacturer's initial setup documentation.

## 2. Workstation and Network Setup

Establishing a reliable, low-latency network connection between the Franka Control unit and the workstation PC is paramount for using the Franka Control Interface (FCI). The FCI operates at a 1 kHz frequency, demanding minimal network delay and jitter to ensure stable robot control.

### 2.1. Workstation Requirements

The workstation PC must meet specific minimum requirements to handle the real-time demands of the FCI 3:

* **Operating System:** Linux with a PREEMPT_RT patched kernel. This real-time kernel patch is mandatory to minimize system latencies and meet the FCI's 1ms communication cycle constraint.3 Ubuntu 20.04 LTS (Focal Fossa) is recommended for compatibility with ROS Noetic and recent libfranka/franka_ros versions.7 Ubuntu 22.04 LTS is recommended for ROS 2 Humble and the latest libfranka.5
* **Network Card:** 100BASE-TX Ethernet card.3 For optimal performance, a direct connection between the workstation and the Control unit is strongly recommended, bypassing intermediate switches or routers.1 A high-performance PCI-Express network card might be necessary if high Round Trip Times (RTT) are observed even with a direct connection.8
* **Processor/System:** While specific CPU requirements aren't listed, the system must be capable of processing the control loop and network communication within the 1ms cycle time. Disabling CPU frequency scaling and other power-saving features is recommended to ensure consistent performance.3

### 2.2. Physical Connection

Ensure the physical connections are correctly made:

* Connect the Arm and Control unit using the provided connecting cable, ensuring it's firmly attached on both sides.1
* Connect the workstation PC's Ethernet port directly to the LAN port of the Control unit (labeled for the shop floor network). Do NOT connect to the LAN port on the Arm itself.1 This direct connection minimizes network latency and potential packet loss, which is critical for FCI performance.3
* Connect the external activation/enabling device(s) to the Arm's base.1 Keep these devices within reach for safety.

### 2.3. Network Configuration (Static IPs Recommended)

A static IP configuration ensures consistent and predictable communication between the workstation and the Control unit. The following configuration is recommended and used throughout this guide 1:

* Control Unit IP: 172.16.0.2 (referred to as `<fci-ip>`)
* Workstation PC IP: 172.16.0.1
* Netmask (both): 24 (equivalent to 255.255.255.0)

**Step 1: Configure Control Unit IP Address 1**

This requires robot system version 4.2.0 or higher.

* **Access Desk:** Power on the robot. Temporarily connect a device (e.g., laptop) to the Arm's LAN port or use the standard Control LAN port if already configured. Open a web browser and navigate to `https://<robot-ip>` (the default might be different, check robot documentation or use network scanning tools if unsure. Some documentation suggests `https://172.16.0.2` 11 or `http://192.168.0.88` 9 depending on initial setup or previous configuration). Accept any security certificate warnings.12 Log in if required (default credentials might be available or set during initial setup 9).
* **Navigate to Settings:** Find the "Settings" menu within the Desk interface (often represented by gears or sliders).
* **Network Settings:** Go to the "Network" section.
* **Set Static IP:** Deselect any "DHCP Client" option. Enter the static IP `172.16.0.2` and Netmask `24`. Apply the changes.
* **Reconnect:** Disconnect the temporary device (if used) and ensure the workstation PC is connected to the Control unit's LAN port.

**Step 2: Configure Workstation PC IP Address (Ubuntu Example) 1**

Using Ubuntu 20.04/18.04/16.04 GUI:

* **Open Network Settings:** Access the network manager applet (usually in the top bar or system settings).
* **Edit Connection:** Select the wired connection corresponding to the Ethernet port connected to the robot's Control unit. Click "Edit" or the settings icon.
* **IPv4 Settings:** Navigate to the "IPv4" tab.
* **Set Manual IP:** Change the "Method" from "Automatic (DHCP)" to "Manual".
* **Add Address:** Click "Add" and enter:

    * Address: `172.16.0.1`
    * Netmask: `24` or `255.255.255.0`
    * Gateway: Leave blank (unless required for other network access via the same interface).

* **Save:** Apply and save the changes.
* **Reconnect:** Disable and re-enable the network connection or restart the network service for the changes to take effect.

Note: Remember to change the method back to "Automatic (DHCP)" if you need to connect this Ethernet port to a standard DHCP network later.

### 2.4. Initial Robot Preparation via Desk

Before using FCI, the robot needs basic preparation via the Desk interface 1:

* **Power On:** Switch on the Control unit.11 The Arm LEDs will flash yellow during boot and turn solid yellow when ready.9
* **Access Desk:** Navigate to `https://<fci-ip>` (e.g., `https://172.16.0.2`) in your workstation browser.11 Log in if necessary.
* **Unlock Joints:** Locate the brakes/joints unlock button (often near the bottom or in the side bar) and click it. You should hear a click from the arm, and the LEDs should turn solid white.9 White LEDs indicate the joints are unlocked and the arm can be guided manually using the buttons near the end-effector.4
* **Activate FCI:** Find the main menu (often three bars or similar) and select "Activate FCI".1 This enables the external control interface. The robot LEDs should turn blue, indicating it's ready to accept external commands via FCI.4 Ensure the robot's operating mode is set to "Execution" if applicable.11

### 2.5. Verify Network Connection

After configuring IPs and activating FCI, verify the connection:

* **Ping Test:** Open a terminal on the workstation and ping the robot's IP address 4:

    ```bash
    ping 172.16.0.2
    ```

    Successful replies indicate basic network connectivity. If this fails, re-check IP settings, cable connections, and firewall rules.4 A common issue is a firewall blocking UDP packets required by FCI; consider disabling the firewall for testing (`sudo ufw disable` on Ubuntu).4

* **libfranka Communication Test:** Use the `echo_robot_state` example provided with libfranka (details on installing libfranka in Section 3.2). Navigate to the libfranka build directory and run 1:

    ```bash
    ./examples/echo_robot_state 172.16.0.2
    ```

    Successful execution will print the current robot state (joint angles, velocities, etc.) to the terminal for a few iterations before exiting. This confirms that the FCI connection is active and libfranka can communicate with the robot. If it fails with "Connection timeout", double-check FCI activation in Desk and the FCI feature file installation.8 If it fails with "UDP receive: Timeout", check firewall settings.4

## 3. Software Installation

With the network established, the next step is to install the necessary software components on the workstation PC: the PREEMPT_RT kernel, libfranka, and the relevant ROS packages (franka_ros for ROS 1 or franka_ros2 for ROS 2).

### 3.1. Install PREEMPT_RT Kernel

A real-time kernel is mandatory for the low-latency communication required by FCI.3

Steps (Ubuntu 20.04 Example with Kernel 5.9.1 - Adapt versions as needed): 7

* **Install Dependencies:**

    ```bash
    sudo apt-get update
    sudo apt-get install build-essential bc curl ca-certificates gnupg2 debhelper dpkg-dev devscripts fakeroot libssl-dev libelf-dev bison flex cpio kmod rsync libncurses-dev liblz4-tool # liblz4-tool might be needed for newer kernels [17]
    ```

* **Choose Kernel Version:** Find your current kernel (`uname -r`). Check the RT patch availability at `https://www.kernel.org/pub/linux/kernel/projects/rt/`. Select a kernel version matching your current major version with an available RT patch. Example using 5.9.1:

    ```bash
    KERNEL_VERSION="5.9.1"
    RT_PATCH_VERSION="5.9.1-rt20" # Find corresponding patch version
    KERNEL_MAJOR=$(echo <span class="math-inline">KERNEL\_VERSION \| cut \-d\. \-f1\)
curl \-SLO \[https\://www\.kernel\.org/pub/linux/kernel/v</span>](https://www.kernel.org/pub/linux/kernel/v$){KERNEL_MAJOR}.x/linux-<span class="math-inline">\{KERNEL\_VERSION\}\.tar\.xz
curl \-SLO \[https\://www\.kernel\.org/pub/linux/kernel/v</span>](https://www.kernel.org/pub/linux/kernel/v$){KERNEL_MAJOR}.x/linux-<span class="math-inline">\{KERNEL\_VERSION\}\.tar\.sign
curl \-SLO \[https\://www\.kernel\.org/pub/linux/kernel/projects/rt/</span>](https://www.kernel.org/pub/linux/kernel/projects/rt/<span class="math-inline">\)\{KERNEL\_VERSION%\.\*\}/patch\-</span>{RT_PATCH_VERSION}.patch.xz
    # curl -SLO [https://www.kernel.org/pub/linux/kernel/projects/rt/<span class="math-inline">\]\(https\://www\.kernel\.org/pub/linux/kernel/projects/rt/</span>){KERNEL_VERSION%.*}/patch-${RT_PATCH_VERSION}.patch.sign # Optional signature

    (Note: Adapt URLs based on chosen versions. Check official Franka docs for tested versions for your Ubuntu release 7)
    ```

* **Verify Downloads (Optional but Recommended):** Use `gpg` to verify signatures if downloaded.

* **Extract and Patch:**

    ```bash
    xz -d linux-<span class="math-inline">\{KERNEL\_VERSION\}\.tar\.xz
xz \-d patch\-</span>{RT_PATCH_VERSION}.patch.xz # If patch downloaded as .xz
    tar xf linux-<span class="math-inline">\{KERNEL\_VERSION\}\.tar
cd linux\-</span>{KERNEL_VERSION}/
    patch -p1 <../patch-${RT_PATCH_VERSION}.patch
    ```

* **Configure Kernel:**

    ```bash
    # Copy current config
    cp /boot/config-$(uname -r).config .config

    # Make menuconfig (Text-based UI)
    # make menuconfig
    # Navigate: General setup ---> Preemption Model ---> Fully Preemptible Kernel (Real-Time)
    # Save and Exit.
    # OR use scripts (as per Franka docs [7]):
    scripts/config --disable PREEMPT_NONE
    scripts/config --disable PREEMPT_VOLUNTARY
    scripts/config --disable PREEMPT
    scripts/config --enable PREEMPT_RT

    # Disable debug info (recommended by Franka docs [7, 17])
    scripts/config --disable DEBUG_INFO
    scripts/config --disable DEBUG_INFO_DWARF_TOOLCHAIN_DEFAULT
    scripts/config --disable DEBUG_KERNEL

    # Disable system keys (recommended by Franka docs [7], some guides suggest editing .config directly [18])
    scripts/config --disable SYSTEM_TRUSTED_KEYS
    scripts/config --disable SYSTEM_REVOCATION_LIST
    # Alternatively: edit .config and set CONFIG_SYSTEM_TRUSTED_KEYS="" [18]

    # Apply changes based on .config
    make olddefconfig # Or 'make oldconfig' for interactive prompts [13, 17]
    ```

* **Compile Kernel:** This takes a significant amount of time. Use `-jN` where N is the number of CPU cores.

    ```bash
    # Using deb-pkg method (recommended by Franka docs [7])
    make -j$(nproc) deb-pkg
    # Alternative method (less common, potentially more complex install [17]):
    # make clean
    # make -j$(nproc)
    # sudo make modules_install
    # sudo make install
    ```

* **Install Kernel Packages:** Install the generated .deb files (headers and image, excluding dbg packages). The `IGNORE_PREEMPT_RT_PRESENCE=1` flag might be needed if you have NVIDIA drivers installed, as they don't officially support RT kernels.7

    ```bash
    cd ..
    sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg -i linux-headers-*.deb linux-image-*.deb
    # Ensure you select the correct .deb files corresponding to your built RT kernel
    ```

* **Verify Installation:**

    * **Reboot:** Restart your computer. During boot, access the GRUB menu (may require holding Shift or Esc). Select "Advanced options for Ubuntu" and choose the kernel with "rt" in its name.13
    * **Check Kernel:** Once booted into the RT kernel, open a terminal and run `uname -a`. The output should include `PREEMPT_RT`.13
    * **Check Realtime Flag:** Verify that `/sys/kernel/realtime` exists and contains the number `1`.13
    * **Troubleshooting Boot Issues:** If the system fails to boot (e.g., black screen with NVIDIA drivers), you might need to adjust GRUB parameters (like blacklisting nouveau and setting `nvidia.modeset=1`) or rebuild initramfs (`sudo update-initramfs -u`) and update GRUB (`sudo update-grub`).17 Disabling "Secure Boot" in UEFI/BIOS is often necessary.8

* **Set Real-time Permissions:** Allow your user account to run processes with real-time priority 4:

    ```bash
    sudo addgroup realtime
    sudo usermod -a -G realtime $USER # Replace $USER if needed
    ```

    Add the following lines to `/etc/security/limits.conf` (requires sudo to edit):

    ```
    @realtime    -   rtprio      99
    @realtime    -   memlock     unlimited
    ```

    Log out and log back in for the group changes to take effect.

### 3.2. Install libfranka

libfranka is the C++ library providing the low-level FCI communication interface.6 It's recommended to build it from source to ensure compatibility and control over the version.

Steps: 6

* **Remove Existing Installations (Important):** Avoid conflicts by removing any previous libfranka installations (e.g., from ROS repos):

    ```bash
    sudo apt remove "*libfranka*"
    sudo apt autoremove
    ```

* **Install Dependencies:**

    ```bash
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev # Base dependencies [6]
    # Additional dependencies for libfranka >= 0.14.0 (Pinocchio) [5, 6]
    sudo apt-get install -y lsb-release curl
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL ...[source](https://github.com/frankaemika/libfranka/blob/main/README.md)
    ```

* **Clone Repository:**

    ```bash
    cd ~ # Or your preferred development directory
    git clone --recurse-submodules [https://github.com/frankaemika/libfranka.git](https://github.com/frankaemika/libfranka.git)
    cd libfranka
    ```

* **Checkout Correct Version (Crucial):** Check the Compatibility Matrix to find the libfranka version compatible with your robot's firmware (System Version). Checkout the corresponding tag 6:

    ```bash
    git tag -l # List available versions
    git checkout <tag_name> # e.g., git checkout 0.9.2
    git submodule update --init --recursive # Ensure submodules match the tag
    ```

    Note: Some guides suggest specific versions like 0.1.0 for older firmware 16 or 0.8.0/0.9.2 for specific ROS 2 ports.20 Always prioritize the official compatibility matrix.

* **Build:**

    ```bash
    mkdir build
    cd build
    # Add -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake if Pinocchio was installed [5, 6]
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. # Add other CMake flags if needed
    make -j$(nproc)
    ```

* **Install (Optional but Recommended):** Installing system-wide or as a Debian package simplifies linking for other projects like franka_ros.

    * System-wide install:

        ```bash
        sudo make install
        sudo ldconfig # Update linker cache
        ```

    * Build Debian Package: 5

        ```bash
        cpack -G DEB
        sudo dpkg -i libfranka*.deb
        ```

* **Link without installing:** If not installing, you may need to add the build directory to `LD_LIBRARY_PATH` in your `.bashrc` 16:

    ```bash
    # Add to ~/.bashrc
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"
    # Then source ~/.bashrc
    ```

### 3.3. Install ROS (Noetic or ROS 2)

Install the desired ROS distribution on your workstation.

**ROS 1 Noetic (on Ubuntu 20.04):** 23

* **Configure Repositories:** Ensure Ubuntu Universe, Restricted, Multiverse are enabled.
* **Add ROS Repository:**

    ```bash
    sudo sh -c 'echo "deb [http://packages.ros.org/ros/ubuntu](http://packages.ros.org/ros/ubuntu) $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

* **Add Keys:**

    ```bash
    sudo apt install curl # if not already installed
    curl -s [https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc](https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc) | sudo apt-key add -
    ```

* **Install ROS:**

    ```bash
    sudo apt update
    sudo apt install ros-noetic-desktop-full # Recommended
    # Or ros-noetic-desktop or ros-noetic-ros-base
    ```

* **Environment Setup:**

    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

* **Install Build Tools & Dependencies:**

    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init # May need sudo apt install python3-rosdep first
    rosdep update
    ```

**ROS 2 (Foxy/Humble on Ubuntu 20.04/22.04):** 25

* **Set Locale:** Ensure UTF-8 locale is set (see ROS 2 install docs 27).
* **Add ROS 2 Repository:**

    ```bash
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

* **Install ROS 2:**

    ```bash
    sudo apt update
    sudo apt upgrade # Recommended
    sudo apt install ros-<distro>-desktop # E.g., ros-humble-desktop (Recommended)
    # Or ros-<distro>-ros-base
    sudo apt install python3-argcomplete # Often needed
    sudo apt install ros-dev-tools # For building packages
    ```

* **Environment Setup:**

    ```bash
    echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

* **Install Build Tools & Dependencies (if needed):**

    ```bash
    sudo apt install python3-colcon-common-extensions python3-rosdep
    sudo rosdep init # If not already done
    rosdep update
    ```

### 3.4. Install franka_ros (for ROS 1 Noetic)

This metapackage integrates libfranka with ROS 1 and ros_control.29

Steps: 7

* **Create/Navigate to Catkin Workspace:**

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    # Ensure ROS environment is sourced
    source /opt/ros/noetic/setup.bash
    catkin init # If using catkin tools, or catkin_init_workspace src if using catkin_make
    ```

* **Clone Repository:**

    ```bash
    cd src
    git clone --recursive [https://github.com/frankaemika/franka_ros.git](https://github.com/frankaemika/franka_ros.git)
    # Optional: Checkout specific version matching libfranka/firmware if needed
    # git checkout <tag_name>
    ```

* **Install Dependencies:** Use rosdep to install dependencies, skipping libfranka as it was built from source.

    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
    ```

* **Build:** Build the workspace using `catkin_make` or `catkin build`. Crucially, tell CMake where to find your custom libfranka build using `Franka_DIR`.7

    ```bash
    # Using catkin_make
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/path/to/your/libfranka/build
    # Or using catkin build
    # catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/path/to/your/libfranka/build
    # catkin build
    ```

    Replace `/path/to/your/libfranka/build` with the actual path to the build directory where you compiled libfranka. If you installed libfranka system-wide, CMake might find it automatically, but explicitly setting `Franka_DIR` ensures the correct version is used.

* **Source Workspace:** Add the workspace to your ROS environment.

    ```bash
    source ~/catkin_ws/devel/setup.bash
    # Add to ~/.bashrc for persistence
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    ```

### 3.5. Install franka_ros2 (for ROS 2 Humble/Foxy)

This metapackage integrates libfranka with ROS 2 and ros2_control.32

Steps (Using Local Build): 28

* **Create/Navigate to Colcon Workspace:**

    ```bash
    mkdir -p ~/franka_ros2_ws/src
    cd ~/franka_ros2_ws
    # Ensure ROS 2 environment is sourced
    source /opt/ros/<distro>/setup.bash
    ```

* **Clone Repositories:** Clone franka_ros2 and its dependencies using the provided `.repos` file.

    ```bash
    git clone [https://github.com/frankaemika/franka_ros2.git](https://github.com/frankaemika/franka_ros2.git) src/franka_ros2
    # Import dependencies
    vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing
    ```

    Note: Ensure the correct branch (e.g., humble) is checked out in franka_ros2 if not cloning the default.

* **Install Dependencies:** Use rosdep to install system dependencies.

    ```bash
    rosdep install --from-paths src --ignore-src --rosdistro <distro> -y
    ```

* **Build:** Build the workspace using colcon.

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

* **Source Workspace:**

    ```bash
    source ~/franka_ros2_ws/install/setup.bash
    # Add to ~/.bashrc for persistence
    echo "source ~/franka_ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

Docker installation is also provided as an alternative in the franka_ros2 README.28

## 4. Launching Franka ROS Interface and Verifying Connection

With the software installed, you can now launch the ROS interface and verify communication using standard ROS tools.

### 4.1. Launching the ROS 1 Interface (franka_control_node)

The `franka_control_node` is the core hardware interface node for ROS 1.29

* **Ensure Robot is Ready:** Power on the robot, unlock joints, and activate FCI via Desk (LEDs should be blue).
* **Source Workspace:** Open a terminal and source your Catkin workspace:

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

* **Launch Node:** Use `roslaunch` with the `franka_control.launch` file 29:

    ```bash
    roslaunch franka_control franka_control.launch \
    robot_ip:=172.16.0.2 \
    load_gripper:=true \
    robot:=panda # Or fr3
    ```

    * `robot_ip`: Mandatory IP of the Control unit.
    * `load_gripper`: (Default: true) Also launches the `franka_gripper_node` if set to true.29
    * `robot`: (Default: panda) Specifies the robot model URDF/config to load.37

    This launch file typically starts:

    * `franka_control_node`: The main hardware interface.
    * `franka_state_controller`: Reads and publishes robot state.29
    * `robot_state_publisher`: Publishes TF transforms based on joint states for visualization.29
    * `franka_gripper_node` (if `load_gripper:=true`).

### 4.2. Verifying ROS 1 Connection

Use standard ROS 1 tools in separate terminals (after sourcing the workspace):

* `rostopic list`: Check for topics like `/franka_state_controller/joint_states`, `/franka_state_controller/franka_states`, `/tf`, and gripper topics (e.g., `/franka_gripper/joint_states`) if loaded.11
* `rostopic echo /franka_state_controller/franka_states`: View the detailed robot state messages being published. Seeing continuous data confirms communication.11
* `rosnode list`: Verify that `/franka_control`, `/franka_state_controller`, `/robot_state_publisher`, and `/franka_gripper` (if loaded) are running.29
* `rviz`: Launch RViz (`rosrun rviz rviz`). Add a `RobotModel` display, setting the "Robot Description" parameter to `robot_description`. Add a `TF` display. You should see the 3D model of the robot reflecting its current pose.29

### 4.3. Launching the ROS 2 Interface

The ROS 2 interface uses `ros2_control` and is typically launched via the `franka_bringup` package.32

* **Ensure Robot is Ready:** Power on, unlock joints, activate FCI (blue LEDs).
* **Source Workspace:** Open a terminal and source your Colcon workspace:

    ```bash
    source ~/franka_ros2_ws/install/setup.bash
    ```

* **Launch Interface:** Use `ros2 launch` 32:

    ```bash
    ros2 launch franka_bringup franka.launch.py \
    robot_ip:=172.16.0.2 \
    arm_id:=fr3 \ # Or panda
    load_gripper:=true \
    use_rviz:=true
    ```

    * `robot_ip`: Mandatory IP of the Control unit.
    * `arm_id`: Identifier for the robot (used for namespacing).
    * `load_gripper`: (Default: true) Loads gripper components.
    * `use_rviz`: (Default: false) Launches RViz for visualization.

    This launch file starts the `ros2_control` node with the `FrankaHardwareInterface`, the `joint_state_broadcaster`, and potentially the `franka_gripper` node and RViz.33

### 4.4. Verifying ROS 2 Connection

Use standard ROS 2 tools in separate terminals (after sourcing the workspace):

* `ros2 topic list`: Look for topics like `/joint_state_broadcaster/joint_states`, `/franka_robot_state_broadcaster/robot_state` (if state broadcaster is loaded separately), `/tf`, and gripper topics (e.g., `/franka_gripper/joint_states`).32
* `ros2 topic echo /joint_state_broadcaster/joint_states`: View the joint state messages. Seeing continuous data confirms communication.32
* `ros2 node list`: Verify that nodes like `/controller_manager`, `/joint_state_broadcaster`, `/robot_state_publisher`, and `/franka_gripper` (if loaded) are running.32
* `rviz2`: If launched (`use_rviz:=true`), RViz should display the robot model. Add `RobotModel` and `TF` displays if not already present.32

Successful verification using these tools confirms that the ROS interface is correctly communicating with the Franka robot via FCI.

## 5. MoveIt! Integration and Basic Motion Planning

MoveIt! is the standard motion planning framework in ROS. The `franka_ros` and `franka_ros2` ecosystems provide configuration packages (`panda_moveit_config` or `franka_fr3_moveit_config`) to easily use MoveIt! with Franka robots for visualization, collision-aware planning, and execution.15

### 5.1. MoveIt! Setup and Launch

**Prerequisites:**

* MoveIt! installed for your ROS distribution (ROS 1 Noetic or ROS 2).
* The appropriate MoveIt! configuration package installed:
    * **ROS 1 Noetic:** `panda_moveit_config`. This package is maintained separately by the ros-planning organization.41 Install it from source into your Catkin workspace or via apt if available (`sudo apt install ros-noetic-panda-moveit-config`).45
    * **ROS 2 Humble/Foxy:** `franka_fr3_moveit_config`. This is typically included within the `franka_ros2` repository or its dependencies.32 Ensure it's built as part of your Colcon workspace.

**Launching MoveIt! with RViz:**

**ROS 1 Noetic:**

* Ensure the `franka_control_node` is NOT running separately if the MoveIt! launch file starts its own controllers. Some MoveIt launch files might start the required drivers/controllers themselves. Check the specific launch file (e.g., `demo.launch` or potentially `franka_control.launch` within `panda_moveit_config`).
* Launch the MoveIt! demo, which includes RViz with the MotionPlanning plugin:

    ```bash
    # Example using demo.launch (starts fake controllers by default)
    roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

    # Example connecting to real robot (if franka_control.launch is configured for MoveIt!)
    # roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true
    ```

    Consult the `panda_moveit_config` documentation for the correct launch file to connect to the real robot. The `franka_control.launch` file within `panda_moveit_config` is often used for connecting to the real hardware.50

**ROS 2 Humble/Foxy:**

* Launch the MoveIt! setup:

    ```bash
    # For real robot
    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2

    # For fake hardware simulation
    # ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
    ```

    This command launches the necessary MoveIt! components and RViz.32 Wait for the "You can start planning now!" message in the terminal.32

### 5.2. Visualizing the Robot in RViz

Once launched, RViz should display:

* **Robot Model:** The 3D model of the Franka arm and gripper (if loaded).
* **MotionPlanning Display:** This panel (usually on the left) provides interfaces for:
    * Selecting the planning group (e.g., "panda_arm", "hand").
    * Setting start and goal states (using interactive markers or predefined poses).
    * Visualizing the planning scene (including collision objects).
    * Invoking the planner (Plan button).
    * Visualizing the planned trajectory.
    * Executing the plan on the robot (Execute button).

Interact with the orange interactive marker representing the end-effector goal pose. Drag it around and observe the robot model in RViz tracking it (if inverse kinematics succeeds).51

### 5.3. Planning Collision-Free Motions

MoveIt! uses the collision geometry defined in `franka_description` (specifically the "coarse" collision model for planning efficiency) and the configured planning scene to generate collision-free paths.29

* **Set Goal State:** In the RViz MotionPlanning display, under the "Planning" tab, set a desired goal state. You can drag the interactive marker or select a predefined pose from the dropdown (if available).
* **Plan:** Click the "Plan" button. MoveIt! will invoke the configured motion planner (e.g., OMPL) to find a collision-free path from the current state to the goal state. The planned trajectory will be visualized

* * **Execute:** If a plan is found and you are connected to the real robot (or fake hardware execution is enabled), click the "Execute" button. MoveIt! will send the trajectory to the appropriate ROS controllers (managed by `franka_control` or `ros2_control`) for execution on the hardware.

###   5.4. Sending Joint Space and Cartesian Space Goals

    You can command the robot programmatically using the MoveIt! interfaces, typically the MoveGroupInterface (C++) or MoveGroupCommander (Python).

    **Joint Space Goals:** Specify target joint angles for the planning group.

    ```python
    # ROS 1 Python Example (MoveGroupCommander)
    import moveit_commander
    #... setup move_group...
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0  # Set target for joint 1
    joint_goal[1] = -0.785  # Set target for joint 2
    #... set other joints...
    move_group.go(joint_goal, wait=True)
    move_group.stop()  # Ensure no residual movement
    ```

    **Cartesian Space Goals:** Specify a target pose (position and orientation) for the end-effector link.

    ```python
    # ROS 1 Python Example (MoveGroupCommander)
    import moveit_commander
    import geometry_msgs.msg
    #... setup move_group...
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.5
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    ```

    **Cartesian Paths (Linear Motion):** Plan a path where the end-effector moves linearly between waypoints. This is useful for approach and retreat motions in pick-and-place.

    ```python
    # ROS 1 Python Example (MoveGroupCommander)
    import moveit_commander
    import geometry_msgs.msg
    import copy
    #... setup move_group...
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= 0.1  # Move down 10cm
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.1  # Move forward 10cm
    waypoints.append(copy.deepcopy(wpose))
    #... add more waypoints...
    (plan, fraction) = move_group.compute_cartesian_path(
                                waypoints,  # waypoints to follow
                                0.01,       # eef_step
                                0.0,        # jump_threshold
    )
    move_group.execute(plan, wait=True)
    ```

    Refer to the MoveIt! tutorials for detailed examples using the C++ and Python interfaces.40 The successful execution of planned motions via RViz or code confirms that MoveIt! is correctly configured and communicating with the Franka ROS interface. A critical aspect is ensuring MoveIt! is configured to use the correct controller manager and controller names that match those provided by `franka_control` or `franka_hardware` (ROS 2). Mismatches here are a common source of execution failures.54

##   6. Implementing a Pick-and-Place Task

    This section details the steps to implement a basic pick-and-place task using ROS (focusing on ROS 1 Noetic with Python for clarity) and MoveIt!, integrating the concepts covered previously.

###   6.1. Defining the Environment (Planning Scene)

    Accurate motion planning requires defining the robot's environment, including obstacles like the table and the object to be manipulated.

* **Identify Planning Frame:** Determine the reference frame for adding objects (e.g., `panda_link0` or `world`). Use `move_group.get_planning_frame()`.
* **Add Collision Objects:** Use the PlanningSceneInterface (Python: `moveit_commander.PlanningSceneInterface`) to add objects.
    * **Table:** Add a box representing the table surface the object rests on.
    * **Object:** Add a box (or cylinder/mesh) representing the object to be picked. Define its pose (position and orientation) relative to the planning frame and its dimensions.

    ```python
    # ROS 1 Python Example (within your pick-and-place node)
    import rospy
    import moveit_commander
    import geometry_msgs.msg
    import time

    #... (Initialize ROS node, RobotCommander, PlanningSceneInterface, MoveGroupCommander)...
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("panda_arm")  # Or your arm group name
    gripper_group = moveit_commander.MoveGroupCommander("hand")  # Or your gripper group name

    planning_frame = move_group.get_planning_frame()  # e.g., "panda_link0"
    eef_link = move_group.get_end_effector_link()

    # Add Table
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = planning_frame
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.5
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = -0.025  # Slightly below base frame origin
    table_name = "table"
    scene.add_box(table_name, table_pose, size=(0.8, 0.8, 0.05))
    rospy.sleep(1.0)  # Allow time for scene update

    # Add Object to Pick
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = planning_frame
    object_pose.pose.orientation.w = 1.0
    object_pose.pose.position.x = 0.4
    object_pose.pose.position.y = 0.0
    object_pose.pose.position.z = 0.025  # Sitting on the table (z=0) + half height
    object_name = "object_to_pick"
    scene.add_box(object_name, object_pose, size=(0.04, 0.04, 0.1))  # Example: 4x4x10cm box
    rospy.sleep(1.0)  # Allow time for scene update
    ```

    Code examples adapted from.53 C++ examples can be found in.53

* **Verify in RViz:** Check the "Planning Scene" display in RViz to confirm the table and object appear correctly.

###   6.2. Planning the Pick-and-Place Sequence

    Define the key poses and motions required for the task.61

* **Define Poses:** Determine the Cartesian coordinates (and orientation) for:
    * `home_pose`: A safe starting/ending configuration (often defined as joint values).
    * `pre_grasp_pose`: A pose slightly above the object.
    * `grasp_pose`: The pose where the end-effector grasps the object.
    * `lift_pose`: A pose directly above the grasp pose after lifting.
    * `pre_place_pose`: A pose slightly above the target placement location.
    * `place_pose`: The pose where the object will be released.
    * `retreat_pose`: A pose safely away from the placed object.
* **Sequence Planning:** Plan the motions between these poses using MoveIt!.
    * **Move to Poses:** Use `move_group.go()` or `move_group.set_pose_target()` followed by `plan()` and `execute()` for reaching pre-grasp, lift, pre-place, and home poses.
    * **Approach/Retreat:** Use `move_group.compute_cartesian_path()` to plan linear motions for approaching the object (from pre-grasp to grasp) and retreating after placing (from place to retreat). This ensures straight-line movement for these critical phases.

###   6.3. Integrating Gripper Control (franka_gripper Actions)

    Command the gripper hardware at the appropriate points in the sequence using the action clients established for `franka_gripper`.29

* **Initialize Action Clients:** (Assuming `franka_gripper_node` is running, see Section 4.1)

    ```python
    # ROS 1 Python Example
    import actionlib
    from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
    from control_msgs.msg import GripperCommandAction, GripperCommandGoal  # For MoveIt compatibility

    # Create clients (do this once during node initialization)
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    gripper_command_client = actionlib.SimpleActionClient('/franka_gripper/gripper_action', GripperCommandAction)

    move_client.wait_for_server(rospy.Duration(5.0))
    grasp_client.wait_for_server(rospy.Duration(5.0))
    gripper_command_client.wait_for_server(rospy.Duration(5.0))
    ```

* **Define Gripper Commands:** Create functions or methods to send open and close commands.

    ```python
    # ROS 1 Python Example
    def open_gripper(wait=True):
        goal = MoveGoal()
        goal.width = 0.08  # Fully open for Panda
        goal.speed = 0.1
        move_client.send_goal(goal)
        if wait:
            move_client.wait_for_result(rospy.Duration(5.0))
        return move_client.get_result()

    def close_gripper(width=0.0, force=20.0, speed=0.05, wait=True):
        # Using GraspAction
        goal = GraspGoal()
        goal.width = width  # Target width when grasping (adjust based on object)
        goal.epsilon.inner = 0.005  # Tolerance
        goal.epsilon.outer = 0.005
        goal.speed = speed
        goal.force = force
        grasp_client.send_goal(goal)

        # # Alternative: Using GripperCommandAction (often used by MoveIt pick pipeline)
        # goal = GripperCommandGoal()
        # goal.command.width = width
        # goal.command.max_effort = force  # max_effort > 0 triggers grasp attempt
        # gripper_command_client.send_goal(goal)

        if wait:
            grasp_client.wait_for_result(rospy.Duration(5.0))  # Adjust client based on method used
        return grasp_client.get_result()  # Adjust client based on method used
    ```

    (Note: The `GraspAction` provides explicit force control and epsilon parameters, while `GripperCommandAction` is the standard MoveIt! interface.29 Choose the one that fits the control strategy. For simple closing, `MoveAction` to width 0 might suffice if force control isn't critical).

* **Integrate into Sequence:** Call `open_gripper()` before the approach motion and `close_gripper()` after reaching the grasp pose. Call `open_gripper()` again after reaching the place pose.

###   6.4. Attaching/Detaching Objects in Planning Scene

    Inform MoveIt! when the object is attached to or detached from the gripper. This is crucial for collision checking during transport.

    ```python
    # ROS 1 Python Example
    def attach_object(object_name, touch_links=None):
        if touch_links is None:
            touch_links = [eef_link, "panda_leftfinger", "panda_rightfinger"]  # Example links
        scene.attach_box(eef_link, object_name, touch_links=touch_links)
        rospy.sleep(1.0)  # Allow time for update

    def detach_object(object_name):
        scene.remove_attached_object(eef_link, name=object_name)
        rospy.sleep(1.0)  # Allow time for update

    def remove_object(object_name):
        scene.remove_world_object(object_name)
        rospy.sleep(1.0)  # Allow time for update
    ```

    (Adapted from 53) Call `attach_object()` after successfully grasping and `detach_object()` after releasing. Optionally call `remove_object()` at the end to clean up the scene.

###   6.5. Example ROS Node Structure (Python - ROS 1)

    ```python
    #!/usr/bin/env python3

    import sys
    import copy
    import rospy
    import moveit_commander
    import moveit_msgs.msg
    import geometry_msgs.msg
    import actionlib
    from math import pi
    from std_msgs.msg import String
    from moveit_commander.conversions import pose_to_list
    from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

    # --- Action Client Functions (as defined in 6.3) ---
    move_client = None
    grasp_client = None

    def open_gripper(wait=True):
        #... (implementation from 6.3)...
        pass

    def close_gripper(width=0.0, force=20.0, speed=0.05, wait=True):
        #... (implementation from 6.3)...
        pass

    # --- Planning Scene Functions (as defined in 6.4) ---
    scene = None
    eef_link = ""

    def attach_object(object_name, touch_links=None):
        #... (implementation from 6.4)...
        pass

    def detach_object(object_name):
        #... (implementation from 6.4)...
        pass

    def remove_object(object_name):
        #... (implementation from 6.4)...
        pass

    def pick_and_place():
        global move_client, grasp_client, scene, eef_link

        # --- Initialization ---
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place_node', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"  # Check your SRDF for correct group name
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()  # e.g., "panda_link8"

        # Action clients for gripper
        move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        if not move_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("MoveAction server not available!")
            return
        if not grasp_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("GraspAction server not available!")
            return

        # Planning Scene Setup
        planning_frame = move_group.get_planning_frame()
        object_name = "object_to_pick"
        table_name = "table"

        # Clean scene
        remove_object(object_name)
        remove_object(table_name)
        detach_object(object_name)  # Ensure object is detached if previously attached

        # Add Table
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = planning_frame
        #... (set table pose and size as in 6.1)...
        scene.add_box(table_name, table_pose, size=(0.8, 0.8, 0.05))
        rospy.sleep(1.0)

        # Add Object
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = planning_frame
        #... (set object pose and size as in 6.1)...
        scene.add_box(object_name, object_pose, size=(0.04, 0.04, 0.1))
        rospy.sleep(1.0)

        # --- Define Poses ---
        # (Define geometry_msgs.msg.Pose for pre_grasp, grasp, lift, pre_place, place, retreat)
        # Example: Grasp pose (adjust orientation and z-offset based on object/gripper)
        grasp_pose = geometry_msgs.msg.Pose()
        grasp_pose.orientation.w = 1.0  # Adjust orientation as needed
        grasp_pose.position.x = 0.4
        grasp_pose.position.y = 0.0
        grasp_pose.position.z = 0.1  # Example: Gripper center slightly above object center

        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.position.z += 0.10  # 10cm above

        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += 0.12  # Lift 12cm

        place_pose = geometry_msgs.msg.Pose()
        place_pose.orientation.w = 1.0
        place_pose.position.x = 0.0
        place_pose.position.y = 0.4
        place_pose.position.z = 0.1  # Same height as grasp

        pre_place_pose = copy.deepcopy(place_pose)
        pre_place_pose.position.zOK, here's the continuation of the Markdown, picking up from where we left off with the Python code example:

Markdown

    += 0.10  # 10cm above

        retreat_pose = copy.deepcopy(place_pose)
        retreat_pose.position.z += 0.12  # Retreat up 12cm

        # --- Execute Sequence ---
        rospy.loginfo("Moving to home pose")
        move_group.set_named_target("ready")  # Assuming 'ready' is defined in SRDF
        move_group.go(wait=True)
        move_group.stop()

        rospy.loginfo("Opening gripper")
        open_gripper()

        rospy.loginfo("Moving to pre-grasp pose")
        move_group.set_pose_target(pre_grasp_pose)
        move_group.go(wait=True)
        move_group.stop()

        rospy.loginfo("Approaching object")
        # Cartesian path for approach
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= 0.10  # Move down 10cm to grasp pose Z
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)
        else:
            rospy.logerr("Approach failed")
            return

        rospy.loginfo("Closing gripper (grasping)")
        close_gripper(width=0.035, force=20)  # Adjust width based on object size

        rospy.loginfo("Attaching object to gripper")
        attach_object(object_name)

        rospy.loginfo("Lifting object")
        # Cartesian path for lift
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z += 0.12  # Lift up 12cm
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)
        else:
            rospy.logerr("Lift failed")
            detach_object(object_name)  # Clean up if lift fails
            return

        rospy.loginfo("Moving to pre-place pose")
        move_group.set_pose_target(pre_place_pose)
        move_group.go(wait=True)
        move_group.stop()

        rospy.loginfo("Lowering object")
        # Cartesian path for lowering
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= 0.10  # Move down 10cm to place pose Z
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)
        else:
            rospy.logerr("Lowering failed")
            # Consider what to do if lowering fails - maybe try again or drop?
            detach_object(object_name)
            return

        rospy.loginfo("Opening gripper (releasing)")
        open_gripper()

        rospy.loginfo("Detaching object")
        detach_object(object_name)

        rospy.loginfo("Retreating from object")
        # Cartesian path for retreat
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z += 0.12  # Retreat up 12cm
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)
        else:
            rospy.logerr("Retreat failed")
            # May not be critical, but log it

        rospy.loginfo("Returning to home pose")
        move_group.set_named_target("ready")
        move_group.go(wait=True)
        move_group.stop()

        # Clean up scene
        remove_object(object_name)
        remove_object(table_name)

        rospy.loginfo("Pick and place complete!")

    if __name__ == '__main__':
        try:
            pick_and_place()
        except rospy.ROSInterruptException:
            pass
        finally:
            moveit_commander.roscpp_shutdown()
    ```

    (This example structure synthesizes information from MoveIt! tutorials 40 and integrates gripper control 29 and planning scene interaction.56 It requires the `moveit_commander` Python package and the `franka_gripper` action messages).

    Executing this node (`rosrun <your_package_name> pick_place_node.py`) after launching MoveIt! and the gripper node should perform the pick-and-place task. A successful pick-and-place operation demonstrates the integration of motion planning, environment representation, and gripper actuation within the ROS framework. The planning scene must accurately reflect the environment, the gripper must be commanded correctly, and the planning scene must be updated (attach/detach) for MoveIt! to correctly account for the object during transport.29

##   7. (Optional) Integrating NVIDIA Xavier NX for RL Policy Execution

    This section outlines the process for integrating an NVIDIA Jetson Xavier NX module to run a Reinforcement Learning (RL) policy that controls the Franka robot via the ROS interface running on the main Control PC.

###   7.1. Setting up the NVIDIA Xavier NX

    The Xavier NX is a powerful embedded system suitable for AI and ML tasks at the edge.66

    **Hardware Setup:** 68

* **Unpack:** Gather the Xavier NX Developer Kit (module, carrier board, power supply).
* **Assemble Case (if applicable):** Follow instructions if using a custom case (e.g., with antennas).68
* **Prepare microSD Card:** A high-speed microSD card (UHS-1, 32GB minimum, 64GB+ recommended) is required for the OS.69
* **Connections:** Connect HDMI/DisplayPort monitor, USB keyboard, USB mouse, and Ethernet (or use Wi-Fi during setup). Insert the microSD card into the slot on the underside of the module.69
* **Power On:** Connect the power supply.68
* **Flashing NVIDIA JetPack:** 73 JetPack bundles the Jetson Linux OS (based on Ubuntu), CUDA Toolkit, cuDNN, TensorRT, and other essential libraries.74 Choose the JetPack version compatible with your desired ROS distribution (e.g., JetPack 5.x for Ubuntu 20.04 -> ROS Noetic/Foxy/Humble 76; JetPack 4.x for Ubuntu 18.04 -> ROS Melodic 78). Be aware of potential firmware compatibility issues between major JetPack versions; upgrading firmware might be required before using a newer JetPack SD card image.73

    **Method 1: SD Card Image (Recommended for Dev Kit):** 68

* Download the appropriate Jetson Xavier NX Developer Kit SD Card Image from the NVIDIA JetPack SDK page.69
* Use a tool like Balena Etcher (Windows/Mac/Linux) or `dd` (Linux) to write the downloaded image file (.zip or .img) to the microSD card.69
* Insert the flashed microSD card into the Xavier NX.
* Boot the device. Follow the on-screen first-boot setup instructions (accept EULA, set language, timezone, username/password, connect to Wi-Fi).69

    **Method 2: NVIDIA SDK Manager (Requires Linux Host PC):** 70

* Install SDK Manager on an Ubuntu 18.04/20.04/22.04 host PC.73
* Connect the Xavier NX to the host PC via the Micro-USB port (Device Mode/Recovery Port).71
* Put the Xavier NX into Force Recovery Mode (consult Dev Kit User Guide - typically involves holding a recovery button while pressing reset/power).79
* Launch SDK Manager (`sdkmanager`) and log in with your NVIDIA Developer account.70
* Select "Jetson" as the product category, "Jetson Xavier NX developer kit" as target hardware, and the desired JetPack version.70
* Follow the SDK Manager steps to download components and flash the Jetson OS. It may also install SDK components (CUDA, etc.) onto the Jetson via the USB connection after the initial flash.70

    **Installing ROS on Jetson:**

    Install ROS 1 Noetic or ROS 2 Humble/Foxy following the standard procedures (Section 3.3), but be mindful of the ARM64 architecture.

* **Using `apt`:** Standard `apt install ros-<distro>-...` might work if ARM64 debs are available in the ROS repositories for your JetPack's Ubuntu version.78 Ensure sources are correctly added.
* **Building from Source:** Often necessary, especially for Noetic on Ubuntu 18.04 (JetPack 4.x) or if specific package versions are needed.83 Follow standard source installation procedures, resolving dependencies using `rosdep` (use `-r` flag to ignore errors for packages that might be provided differently on Jetson).83
* **Docker Containers:** A highly recommended approach to manage dependencies and avoid host OS conflicts.82 NVIDIA provides base L4T containers, and projects like `jetson-containers` offer pre-built ROS/ML containers.82

    **Installing ML/RL Dependencies:**

    Install necessary Python packages and ML frameworks.

    ```bash
    sudo apt install python3-pip
    pip3 install -U pip numpy
    ```

    * **TensorFlow/PyTorch:** Install NVIDIA's optimized wheels for Jetson for best performance. Download links are available on NVIDIA's developer forums or documentation.66

        ```bash
        # Example for PyTorch (check NVIDIA docs for correct URL/filename for your JetPack)
        wget <url_to_pytorch_wheel>
        sudo apt-get install libopenblas-base libopenmpi-dev libomp-dev
        pip3 install <pytorch_wheel_filename>.whl
        # Install torchvision matching the PyTorch version
        sudo apt install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
        git clone --branch <version_tag> https://github.com/pytorch/vision torchvision
        cd torchvision
        export BUILD_VERSION=<torchvision_version>  # e.g., 0.12.0
        python3 setup.py install --user
        cd ..
        ```

    * **RL Libraries:** Install libraries like OpenAI Gym, Stable-Baselines3, etc., using `pip3`.

        ```bash
        pip3 install gym stable-baselines3
        ```

    * **Monitoring:** Install `jetson-stats` for monitoring performance (`sudo -H pip3 install -U jetson-stats`).92 Enable max performance mode if needed (`sudo nvpmodel -m 0` or appropriate mode ID for Xavier NX, `sudo jetson_clocks`).79

###   7.2. Network Architecture for Multi-Machine ROS Communication

    The Xavier NX (running the RL policy) and the Control PC (running the Franka ROS driver) must communicate over the network.

* **Physical Network:** Connect both machines to the same Local Area Network (LAN) via Ethernet cables to a common switch/router, or ensure they are on the same Wi-Fi network.94 Ethernet is generally preferred for lower latency and stability.
* **IP Configuration:** Ensure both machines have IP addresses on the same subnet. Static IPs are recommended for predictability. The Control PC already has 172.16.0.1 (Section 2.3). Assign a static IP to the Xavier NX on the same subnet (e.g., 172.16.0.3) using its network settings interface (similar to Section 2.3, Step 2).
* **Verify Connectivity:** Use `ping` from each machine to the other's IP address.95

    ```bash
    # On Xavier NX
    ping 172.16.0.1
    # On Control PC
    ping 172.16.0.3
    ```

    If pings fail, check IP/subnet settings, physical connections, and firewalls.

* **ROS Network Setup:**

    * **ROS 1 (Noetic):** 94

        * **Master:** Run `roscore` on one machine (typically the Control PC).
        * **Environment Variables:** On both machines, set the following in `~/.bashrc` (or `export` in each terminal):

            ```bash
            export ROS_MASTER_URI=http://<control_pc_ip>:11311  # e.g., http://172.16.0.1:11311
            export ROS_IP=<machine_own_ip>  # e.g., 172.16.0.1 on Control PC, 172.16.0.3 on Xavier
            # Alternatively, use ROS_HOSTNAME if DNS is configured
            # export ROS_HOSTNAME=<machine_hostname>
            ```

        * **Hostname Resolution:** Ensure each machine can resolve the other's hostname if using `ROS_HOSTNAME`. This might require editing `/etc/hosts` on both machines to add entries like `172.16.0.1 control_pc_hostname` and `172.16.0.3 xavier_hostname`.11 Using `ROS_IP` avoids potential DNS issues.
        * **Verification:** Run `roscore` on the Control PC. On the Xavier, run `rostopic list`. You should see the standard ROS topics.

    * **ROS 2 (Humble/Foxy):** 95

        * **Default Discovery (Multicast):** If both machines are on the same simple subnet and multicast UDP is allowed, nodes should discover each other automatically without specific configuration.95
        * **ROS_DOMAIN_ID:** Nodes will only communicate if they share the same `ROS_DOMAIN_ID`. The default is 0. To isolate this system or if using multiple ROS 2 systems on the same network, set the same ID (e.g., 5) on both machines 95:

            ```bash
            # Set in ~/.bashrc or export in each terminal on both machines
            export ROS_DOMAIN_ID=5
            ```

        * **Fast DDS Discovery Server (Recommended for Robustness):** If multicast is unreliable (e.g., Wi-Fi, complex networks), use the Discovery Server.102

            * Start server on one machine (e.g., Control PC): `fastdds discovery --server-id 0 --ip-address 172.16.0.1 --port 11811`
            * Configure clients (both machines) to use the server:

                ```bash
                # Set in ~/.bashrc or export in each terminal on both machines
                export ROS_DISCOVERY_SERVER="172.16.0.1:11811"
                ```

        * **Verification:** Launch a talker node on one machine (`ros2 run demo_nodes_cpp talker`) and a listener node on the other (`ros2 run demo_nodes_cpp listener`). They should communicate if the network and domain ID/discovery server are set correctly.

    * **ROS 1 / ROS 2 Bridge:** 104 If the Control PC runs ROS 1 (franka_ros) and the Xavier runs ROS 2, the `ros1_bridge` package is required.

        * **Setup:** Requires a machine (can be the Control PC or Xavier, if capable) running Ubuntu 20.04 with both ROS 1 Noetic and ROS 2 Foxy installed.106
        * **Installation:** Install `ros-foxy-ros1-bridge` via `apt` or build from source.106
        * **Execution:** Source both ROS 1 and ROS 2 environments, then run the bridge:

            ```bash
            source /opt/ros/noetic/setup.bash
            source /opt/ros/foxy/setup.bash
            # Dynamic bridge (bridges all compatible topics/services)
            ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
            # Static bridge (uses YAML config file for specific topics/services)
            # rosparam load <config.yaml> && ros2 run ros1_bridge parameter_bridge
            ```

            This bridge allows topics published on the ROS 1 network (e.g., robot state from franka_ros) to be subscribed to on the ROS 2 network (by the RL policy on Xavier), and vice-versa for commands.

    Establishing reliable network communication is fundamental. Issues often stem from incorrect IP/URI settings, mismatched domain IDs, firewalls blocking discovery/communication, or DNS problems.97

###   7.3. RL Policy Execution Architecture

    A common architecture separates the real-time robot control from the potentially non-real-time, computationally intensive RL policy learning and execution.

* **Control PC (Real-time Domain):**
    * Runs Ubuntu with PREEMPT_RT kernel.
    * Runs libfranka and franka_ros/franka_ros2.
    * Connects directly to the robot via Ethernet/FCI.
    * Handles the 1kHz FCI loop, sending low-level commands (torque, position, velocity) received via ROS and publishing robot state.
    * May run MoveIt! nodes if high-level commands are used.
* **Xavier NX (Policy/Learning Domain):**
    * Runs JetPack (Ubuntu without RT kernel typically).
    * Runs the RL framework (e.g., PyTorch, TensorFlow) and the trained policy.
    * Runs a ROS node that:
        * Subscribes to robot state topics (e.g., `/joint_states`, `/franka_states`) published by the Control PC.
        * Processes the state (observation for the RL policy).
        * Computes the next action using the RL policy.
        * Publishes the action as a command (e.g., target joint velocities, end-effector pose goal) via ROS topics, services, or actions to the Control PC.
* **ROS Network (Communication Bridge):**
    * Connects the Control PC and Xavier NX.
    * Transmits state information from Control PC to Xavier.
    * Transmits action commands from Xavier to Control PC.
    * Uses ROS 1, ROS 2, or ROS 1/2 Bridge depending on the setup.

    **Command Interface Options between Xavier and Control PC:**

* **High-Level (MoveIt! Actions):**
    * **Xavier:** Sends goal poses (geometry_msgs/PoseStamped) or named configurations (e.g., "ready") to the MoveIt! action server (e.g., `/execute_trajectory` for ROS 1, `/execute_trajectory` or similar for ROS 2) likely running on the Control PC.
    * **Control PC:** MoveIt! plans and executes the motion using the underlying franka_ros/franka_ros2 controllers.
    * **Pros:** Simpler RL action space (target poses). Leverages MoveIt!'s collision avoidance.
    * **Cons:** Higher latency due to planning overhead. Less suitable for reactive control.
* **Low-Level (Direct Controller Commands):**
    * **Xavier:** Directly publishes commands to the topics accepted by the active ros_control/ros2_control controllers running via franka_control/franka_hardware. Examples:
        * EffortJointInterface (Torque commands) 29
        * VelocityJointInterface (Joint velocity commands) 29
        * PositionJointInterface (Joint position commands) 29
        * FrankaPoseCartesianInterface / FrankaVelocityCartesianInterface (Cartesian pose/velocity commands) 29 (ROS 1 specific interfaces, ROS 2 uses standard interfaces)
    * **Control PC:** The controller receives the command and passes it (potentially after some processing like PID control) to libfranka for execution within the FCI loop.
    * **Pros:** Lower latency, direct control suitable for RL policies learning control signals.
    * **Cons:** RL policy needs to generate stable control commands. Collision avoidance must be handled explicitly by the policy or safety mechanisms.

    **Data Flow Example (Low-Level Velocity Control):**

* State: Robot -> FCI -> Control PC (franka_ros) -> /joint_states (Topic) -> ROS Network -> Xavier (RL Node subscribes).
* Observation: Xavier RL Node processes /joint_states data.
* Policy Action: RL Policy computes target joint velocities.
* Command: Xavier (RL Node) -> /joint_velocity_controller/command (Topic) -> ROS Network -> Control PC (Velocity Controller subscribes) -> libfranka -> FCI -> Robot.

    **Considerations:**

* Latency: Network and processing delays between Xavier's policy output and robot actuation must be considered in the RL training and control loop design.
* Simulation: Training RL policies directly on physical hardware is often impractical and unsafe. Use simulators like Gazebo (franka_gazebo 32) or NVIDIA Isaac Sim 108 integrated with ROS/ROS 2 for development and training before deploying to the real robot. Frameworks like ros_gazebo_gym facilitate this.110

    This distributed architecture effectively leverages the strengths of each platform: the Control PC's real-time capability for direct robot interface and the Xavier NX's powerful GPU acceleration for AI/RL computations, with ROS providing the necessary communication infrastructure.89

##   8. Troubleshooting and Resources

    This section provides guidance on resolving common issues and lists valuable resources for further help.

## 8.1. Common Issues and Solutions

Refer to the table below for common errors encountered during setup and operation.

Okay, here is the table you requested in Markdown format:

| Error Message / Symptom | Possible Cause(s) | Solution / Verification Steps |Relevant Source(s) |
| :----------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :----------------- |
| Cannot boot realtime kernel: "Invalid Signature" | UEFI Secure Boot is enabled. | Disable "Secure Boot" in UEFI/BIOS settings. | [8, 11] |
| `libfranka` executable fails: "Connection timeout" | FCI mode not activated in Desk; Incorrect network connection (e.g., connected to Arm port); Robot not reachable (IP issue); FCI Feature file not installed. | Ensure FCI is activated in Desk (System \>= 4.2.0). Verify direct Ethernet connection to Control unit LAN port. Check IP configuration and `ping <fci-ip>`. Verify FCI feature is listed in Desk -\> Settings -\> System -\> Installed Features. Contact [email address removed] if feature is missing. | [8] |
| Motion stops: "Discontinuities detected" or "communication\_constraints\_violation" | Commanded values exceed limits; Network latency/packet loss; Non-optimized code; System power-saving features active. | Compile all code with optimizations (`-DCMAKE_BUILD_TYPE=Release`). Ensure direct Ethernet connection to Control. Run network bandwidth/jitter test (`ping -i 0.001`, `communication_test`). Ensure `franka::Robot` uses `RealtimeConfig::kEnforce`. Disable CPU frequency scaling and other power-saving modes in OS/BIOS. | [3, 8] |
| Robot not reachable | Incorrect IP configuration; Network cable issue; Firewall blocking basic connectivity. | Verify static IP settings on both Control unit and workstation. Check Ethernet cable. Run `ping <fci-ip>`. | [8] |
| `libfranka` executable fails: "UDP receive: Timeout" | Workstation firewall blocking incoming UDP packets. | Check firewall rules (`sudo iptables -L`). Temporarily disable firewall (`sudo ufw disable`) for testing. Ensure UDP port used by FCI (check Franka docs) is allowed. | [4, 8] |
| `libfranka` executable fails: "Incompatible Library Version" | `libfranka` version mismatch with robot firmware (System Version). | Check error message for server version. Consult the [Compatibility Matrix](https://frankaemika.github.io/docs/compatibility.html#compatibility-libfranka) and install/build the correct `libfranka` version for your robot's firmware. Rebuild dependent packages (e.g., `franka_ros`). | [8, 20, 113] |
| `libfranka` executable fails: "command rejected/preempted due to activated safety function\!" | Safety function(s) defined in Desk/Watchman are active and conflict with FCI commands. | Review safety configuration in Desk (Watchman). Disable or delete conflicting safety rules if only FCI control is intended for the application. Note: Reading state is still possible. | [8, 114] |
| Realtime scheduling error: "Operation not permitted" | User running the control program is not in the `realtime` group or permissions are not set correctly. | Ensure user is added to `realtime` group (`sudo usermod -a -G realtime $USER`). Verify `/etc/security/limits.conf` contains correct entries for `@realtime`. Log out and log back in. | [115] |
| Multi-machine ROS issues (nodes don't connect, topics missing) | Incorrect `ROS_MASTER_URI` (ROS 1); Incorrect `ROS_IP`/`ROS_HOSTNAME` (ROS 1); Mismatched `ROS_DOMAIN_ID` (ROS 2); Network connectivity/firewall issues; DNS resolution problems (if using hostnames). | **ROS 1:** Verify `ROS_MASTER_URI` points to `roscore` machine IP/hostname. Verify `ROS_IP` is set to each machine's own IP. Check `/etc/hosts` if using hostnames. **ROS 2:** Ensure same `ROS_DOMAIN_ID` on all machines. Check firewall allows UDP multicast (default discovery) or connectivity to Discovery Server IP/port. Use `ping` to test basic connectivity. | [97, 98, 102] |
| Build errors (`franka_ros`, `franka_ros2`) | Missing dependencies; Incorrect `libfranka` version/path specified during build; Compiler issues. | Run `rosdep install` again. Ensure correct `libfranka` version is checked out and built. Verify `-DFranka_DIR` points to the correct `libfranka` build directory during `catkin_make`/`colcon build`. Check compiler compatibility. | [7, 116, 117, 118] |
| Gripper `GraspAction` closes fully instead of grasping at `width`. | Default `epsilon` values might be too small or zero if goal message not fully initialized; Potential issue in specific `franka_ros` versions. | Explicitly set `epsilon.inner` and `epsilon.outer` in the `GraspGoal`. Consider using `MoveAction` to a slightly smaller width if force control is not needed, or monitor feedback. Check GitHub issues for version-specific bugs.[119] | [119, 120] |
| MoveIt\! execution fails ("Unable to identify controllers", "Failed to start controllers") | MoveIt\! controller configuration mismatch with controllers provided by `franka_control`/`franka_hardware`; Controller manager issues. | Verify controller names and types in MoveIt\! config files (`controllers.yaml`) match those loaded by the Franka ROS interface launch file. Ensure the correct controller manager is specified. Check `ros_control`/`ros2_control` status and logs. | [54, 117] |
## 8.2. Official Franka Emika Support & Community

**Official Support Email:** `support@franka.de`. ${8}$ Include your robot's serial number for hardware-specific issues. ${8}$

**Research Contact:** `research@franka.de` (for research-specific inquiries or community contributions). ${122}$

**Franka Community Forum:** [www.franka-community.de](www.franka-community.de) - An interactive platform for user discussions, questions, and shared insights. ${123}$

**Franka World:** [https://franka.world/](https://franka.world/) - Online platform for managing robots, accessing software (like MATLAB toolbox), and potentially community features. ${15}$

**Main Website:** [franka.de](https://franka.de) / [frankaemika.com](https://frankaemika.com). ${122}$

## 8.3. ROS Community Resources

**Robotics Stack Exchange:** The primary Q&A site for ROS (migrated from ROS Answers). ${130}$[https://robotics.stackexchange.com/](https://robotics.stackexchange.com/)

**ROS Discourse:** Official forum for news, announcements, discussions, and working groups. ${131}$[https://discourse.ros.org/](https://discourse.ros.org/)

**ROS Wiki:** Official documentation for ROS packages and concepts. ${24}$[http://wiki.ros.org/](http://wiki.ros.org/)

**ROS Index:** Searchable index of ROS packages. ${42}$[https://index.ros.org/](https://index.ros.org/)

**Reddit r/ROS:** Community discussion forum. ${131}$

## 8.4. MoveIt! Support Resources

**MoveIt Website & Support Page:** [https://moveit.ai/](https://moveit.ai/), Support: [https://moveit.ai/support/](https://moveit.ai/support/). ${140}$

**MoveIt Tutorials:** Official tutorials (check for latest ROS distro). ${40}$

**GitHub Issues:** Report bugs or issues on the MoveIt repositories. ${141}$[https://github.com/moveit/moveit/issues](https://github.com/moveit/moveit/issues)

**ROS Answers/Stack Exchange:** Use the `moveit` tag. ${141}$

**Discord Channel:** Community chat. ${141}$

## 8.5. Key GitHub Repositories

**Franka Emika:**
[`libfranka`](https://github.com/frankaemika/libfranka) ${6}$ (Issues: ${143}$)
[`franka_ros`](https://github.com/frankaemika/franka_ros) ${15}$ (Issues: ${135}$)
[`franka_ros2`](https://github.com/frankaemika/franka_ros2) ${15}$ (Issues: ${144}$)

**MoveIt!:**
[`moveit`](https://github.com/moveit/moveit) ${141}$
`panda_moveit_config`:
