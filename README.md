# [5dpo_omnijoy_driver](https://github.com/5dpo/5dpo_omnijoy_driver/)

**Version 0.0.0**

This repository implements a ROS driver to control an omnidirectional robot
(but also compatible with other steering geometries) with a joystick controller.

**With this version, it is possible to do:**

- Joystick control of a mobile robot

**The next version will add these features:**

No further development is foreseen for this package.

## ROS

**foxy**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

**noetic**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS 1 Noetic](https://wiki.ros.org/noetic/)

### Dependencies

- [rclcpp](https://index.ros.org/r/rclcpp/)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs/)
- [joy](https://index.ros.org/p/joy/)
- [sensor_msgs](https://index.ros.org/p/sensor_msgs/)

### Parameters

- axis_linear_x (`int = 1`): index number of x linear axis (`/joy.axes`)
- axis_linear_y (`int = 2`): index number of y linear axis (`/joy.axes`)
- axis_angular (`int = 0`): index number of angular linear axis (`/joy.axes`)
- axis_deadman (`int = 4`): index number of button for deadman switch
  (`/joy.buttons`)
- axis_turbo (`int = 5`): index number of button for turbo (`/joy.buttons`)
- axis_turbo_up (`int = 6`): index number of button for increasing boost
  (`/joy.buttons`)
- axis_turbo_down (`int = 7`): index number of button for decreasing boost
  (`/joy.buttons`)
- scale_linear (`double = 0.1`): linear scale to normalize (m/s)
- scale_angular (`double = 0.2`): angular scale to normalize (rad/s)
- turbo_scale_linear (`double = 0.2`): use this scale instead of `scale_linear`
  when pressing the turbo button (m/s)
- turbo_max_scale_linear (`double = 0.4`): maximum linear turbo scale (m/s)
- turbo_scale_angular (`double = 0.4`): use this scale instead of
  `scale_angular` when pressing the turbo button (rad/s)
- turbo_max_scale_angular (`double = 0.8`): maximum angular turbo scale (rad/s)

In order to know the index numbers of your controller, go to
[joy](https://wiki.ros.org/joy) documentation, _5. Application_, to see the
options available for those parameters.

### Subscribes

- joy
([`Joy.msg`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html))

### Publishes

- cmd_vel
  ([Twist.msg](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# ROS 2
source /opt/ros/foxy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src

# Clone the repository
cd ~/ros2_ws/src
git clone git@github.com:5dpo/5dpo_omnijoy_driver.git

# Build
colcon build
source install/setup.bash
```

### Launch

```sh
roslaunch sdpo_omnijoy_driver sdpo_omnijoy_driver_logif710.launch.xml
```

## Acknowledges

- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)

## Contacts

If you have any questions or you want to know more about this work, please
contact any member of the 5dpo Robotics Team.
