# Packages for simulating Leo Rover in ROS2 and Gazebo

ROS 2 version | Gazebo version | Branch | Binaries hosted at
-- | -- | -- | --
Humble | Fortress | [humble](https://github.com/LeoRover/leo_simulator-ros2/tree/humble) | https://packages.ros.org
Humble | Garden | [humble](https://github.com/LeoRover/leo_simulator-ros2/tree/humble) | only from source
Humble | Harmonic | [humble](https://github.com/LeoRover/leo_simulator-ros2/tree/humble) | only from source
Jazzy | Harmonic | [ros2](https://github.com/LeoRover/leo_simulator-ros2/tree/ros2) | https://packages.ros.org
Rolling | Ionic | [ros2](https://github.com/LeoRover/leo_simulator-ros2/tree/ros2) | https://packages.ros.org

## Packages
* `leo_simulator` - Metapackage which provides all other packages.
* `leo_gz_bringup` - Launch files for starting simulation and adding Leo Rover inside a simulated world.
* `leo_gz_plugins` - Gazebo plugins for simulated Leo Rover.
* `leo_gz_worlds` - Custom simulation worlds.

## Install

This branch supports ROS Rolling. See above for other ROS versions.

### Binaries

Rolling binaries are available for Gazebo Harmonic. They are hosted at https://packages.ros.org.

1. Make sure you've [installed ROS](https://docs.ros.org/en/rolling/Installation.html) from binary repositories. 

1. Install `ros-<distro>-leo-simulator` package. On Ubuntu with ROS Rolling:
   ```
   sudo apt install ros-rolling-leo-simulator
   ```

### From source

1. Make sure you've [installed ROS](https://docs.ros.org/en/rolling/Installation.html) from binary repositories. 
1. Setup a [colcon workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html):
   ```
   mkdir -p ~/ws/src
   ```
1. Clone this repository into the workspace:
   ```
   cd ~/ws/src
   git clone https://github.com/LeoRover/leo_simulator-ros2 -b <distro>
   ```
1. Install dependencies using [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#how-do-i-use-the-rosdep-tool):
   ```
   cd ~/ws
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src -y --ignore-src -r
   ```
1. Build the project and source the workspace:
   ```
   colcon build --symlink-install
   source install/setup.bash
   ```
### Run Simulation
Run a simulation world with leo rover:

```
ros2 launch leo_gz_bringup leo_gz.launch.py
```

Launch agruments:
* `sim_world` (default: `leo_empty.sdf`) - The Gazebo world to use. Refer to the [leo_gz_worlds](https://github.com/LeoRover/leo_simulator-ros2/tree/ros2/leo_gz_worlds) package for available worlds.
* `robot_ns` (default: `""`) - Robot namespace
    
Example:

```
ros2 launch leo_gz_bringup leo_gz.launch.py sim_world:=marsyard2021.sdf robot_ns:=leo1
```

Add another leo rover to an already running gazebo world:

```
ros2 launch leo_gz_bringup spawn_robot.launch.py robot_ns:=leo2
```
