# ros_control
[Wiki](http://wiki.ros.org/ros_control)

The ros_control framework provides the capability to implement and manage robot controllers. It implements abstractions on hardware interfaces with minimal assumptions on hardware or operating system. The output is applied to the real robot or to its simulation in Gazebo by using a simple Gazebo plugin adapter.

The stack elements can be grouped in:
* The controller and the control manager (blue and yellow blocks)
* The hw abstraction layer (grey and orange blocks) which is composed by:
    * ros_control interfaces (hw resources interface layer)
    * hardware interfaces which contains the hardware drivers.


## Controllers

### Available controllers 
[Wiki](http://wiki.ros.org/ros_controllers)

* position_controllers
* velocity_controllers
* effort_controller
* joint_trajectory_controllers
* joint_state_controller


### Controller manager
It provides the infrastructure to interact with controllers:
* load
* unload
* start
* stop
* spawner (load + start)
* kill (stop + unload)


The intaraction can be done in several ways:
* CLI
    rosrun controller_manager controller_manager <command> <controller_name>

* launch file
``` xml
    <launch>
        <node pkg="controller_manager" type="command" args="controller_name" />
    </launch>
```

* rqt
    rosrun rqt_controller_manager rqt_controller_manager

* ROS API (services)
    * service request
        * controller_manager/load_controller
        * controller_manager/unload_controller
        * controller_manager/switch_controller
    * service respond
        * controller_manager/list_controllers
        * controller_manager/list_controller_types
        * controller_manager/reload_controller_libraries


### Configuring and launching controllers

Controllers are usually defined with .yaml files in a "conf" folder (parameter server). Then they can be called in a launch file:

``` xml
    <!-- Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find robot_control)/config/robot_control.yaml" command="load"/>
```


## Hardware Abstraction Layer

### ros_control interface
It links the controllers with the "hw interface".



### Hw interface
To control a given robot thorugh ros_control, it should be implemented a class which inherit from the RobotHW class (for real implementation) or from RobotHWSim class (for simulation).



# Gazebo with ros_control (gazebo_ros_control)

* filename="libgazebo_ros_contro.so"





