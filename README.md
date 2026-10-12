# cartpole-ros2

This is a ROS 2 simulation and control package for a cartpole system featuring an LQR controller written in C++ and Gazebo Jetty integration.

## ✨ Features

- **`cartpole_control`**: C++ controllers implementing feedback/balancing logic using Eigen3 library.
- **`cartpole_description`**: URDF/Xacro kinematic and dynamic models of the cartpole system.
- **`cartpole_gazebo`**: Gazebo simulation world, SDF/URDF spawning configs and `ros_gz_bridge` setup.

## 📋 Prerequisites

- **ROS 2** (e.g. Jazzy, Lyrical)
- **Gazebo Sim** (Harmonic or Jetty, depending on your ROS 2 distro)
- **Eigen3** library

## 📦 Installation

1. Clone the repository into your ROS 2 workspace `src` directory:

```bash
cd ~/your_ws/src

git clone https://github.com/readattached/cartpole-ros2.git
```

2. Install dependencies using rosdep:

```bash
cd ~/your_ws

rosdep install --from-paths src -y --ignore-src
```

3. Build the workspace with colcon:

```bash
colcon build -- packages-select cartpole_control cartpole_description cartpole_gazebo
```

4. Source your workspace:

```bash
source install/setup.bash
```

## 💻 Usage

To launch the simulation and spin up the cartpole simulation environment in Gazebo, run:

```bash
ros2 launch cartpole_gazebo cartpole.launch.py
```

Open a new terminal or split your current terminal and run:

```bash
ros2 launch cartpole_control lqr_balancer.launch.py
```

## License

This project is open-source and licensed under the Apache License 2.0.

