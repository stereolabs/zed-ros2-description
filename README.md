<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
   ROS 2 Interfaces
</h1>

The `zed-ros2-description` repository installs the `zed_description` ROS2 package which defines the ZED Camera models to be used with the  [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and [ZED ROS2 Examples](https://github.com/stereolabs/zed-ros2-examples).

**Note:** This package does not require CUDA and can be used to receive ZED data on ROS 2 machines without an NVIDIA GPU.

## Install the package from the binaries for ROS 2 Humble

The package `zed_description` is available in binary format in the official Humble repository.

`sudo apt install ros-humble-zed-description`

## Install the package from the source code

You can install the `zed_description` package from the source code to obtain the latest updates or for distributions other than Humble (e.g. ROS 2 Foxy).

### Build the repository

#### Dependencies

The `zed_description` is a colcon package. It depends on the following ROS 2 packages:

- ament_cmake_auto
- builtin_interfaces
- xacro
- rosidl_default_generators
- rosidl_default_runtime
- ament_lint_auto
- ament_cmake_copyright
- ament_cmake_cppcheck
- ament_cmake_lint_cmake
- ament_cmake_pep257
- ament_cmake_uncrustify
- ament_cmake_xmllint

#### Clone and build

Open a terminal, clone the repository, update the dependencies, and build the packages:

```bash
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros2-description.git
cd ../
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

> :pushpin: **Note**: If the command `rosdep` is missing, you can install it using the following method:
>
> `sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential`
>
> :pushpin: **Note**: The option `--symlink-install` is important, it allows using symlinks instead of copying files to the ROS2 folders during the installation, where possible. Each package in ROS2 must be installed and all the files used by the > nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for > all the files that don't need to be compiled (Python scripts, configurations, etc.).
>
> :pushpin: **Note**: If you are using a different console interface like `zsh`, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc and source ~/.zshrc`.

## Visualize a camera in Rviz2

You can visualize the 3D model of each camera in Rviz2 (without starting a ZED ROS2 Wrapper node) by using this command:

```bash
ros2 launch zed_description zed_viz.launch.py camera_model:=<camera_model>
```

where `camera_model` can be one of 'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxhdrmini', 'zedxhdr', 'zedxhdrmax','zedxonegs','zedxone4k','zedxonehdr'.

> :pushpin: **Note**: This command must be used only to debug the correct URDF behaviors. It does not start a working ZED node.
