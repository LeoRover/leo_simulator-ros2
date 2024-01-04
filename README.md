# leo_simulator-ros2
Packages to simulate Leo Rover in ROS 2.
* leo_simulator - Metapackage for this repository.
* leo_gz_bringup - Launch files for starting simulation and adding Leo Rover inside a simulated world.
* leo_gz_plugins - Gazebo plugins for simulated Leo Rover
* leo_gz_worlds - Custom simulation worlds
  
### Building the simulation

1. Setup a [colcon workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. Install [ros_gz](https://github.com/gazebosim/ros_gz) package (Gazebo Fortress or Garden)
3. Clone [leo_common-ros2](https://github.com/LeoRover/leo_common-ros2) repository into the workspace:
   ```
   cd your_workspace_name/src
   git clone https://github.com/LeoRover/leo_common-ros2
   ```
4. Clone this repository into the workspace:
   ```
   git clone https://github.com/LeoRover/leo_simulator-ros2
   ```
5. If using Gazebo Garden (skip this step if using Fortress):
   ```
   export GZ_VERSION=garden
   ```
7. Install dependencies using [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#how-do-i-use-the-rosdep-tool):
   ```
   cd ..
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src -y --ignore-src
   ```
8. Build the project and source the workspace:
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
  * sim_world: Path to the Gazebo world file 
  * robot_ns: Robot namespace
    
  Example:
   ```
   ros2 launch leo_gz_bringup leo_gz.launch.py sim_world:=~/colcon_ws/src/leo_simulator-ros2/leo_gz_worlds/worlds/marsyard2021.sdf robot_ns:=your_namespace
   ```
Add another leo rover to an already running gazebo world:
   ```
   ros2 launch leo_gz_bringup spawn_robot.launch.py robot_ns:=your_namespace
   ```
