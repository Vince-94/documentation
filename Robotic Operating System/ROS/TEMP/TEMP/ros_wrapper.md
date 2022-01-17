# ROS wrapper

A ROS wrapper is simply a node that you create on top of a piece of (non-ROS) code, in order to create ROS interfaces for this code.
The driver’s interface allows you to use the hardware without knowing the specifics of the communication (ex: SPI, I2C, Serial, Websockets, …).

Staps:
1. Write the driver/library with no ROS dependancies
2. Create ROS wrapper on top of the non-ROS driver
3. Paackaging all


## Writing non-ROS driver
The driver is the part that is completely independent from ROS in order to:
- separate the implementation and the interface
- use the driver for other applications that do not use ROS
- share the work with who does not know ROS


### motor_driver.py (example)

``` python3
class MotorDriver:

    def __init__(self, max_speed=10):
        # Init communication, set default settings, ...

        ...

    def set_speed(self, speed):
        # Give a speed that the motor will try to reach.
        ...

    def stop(self):
        # Set speed to 0 and thus stop the motor
        ...

    def get_speed(self):
        # Return current speed
        ...

    def get_status(self):
        # Get hardware information from the motor
        ...

```




## Create ROS wrapper
The ROS wrapper allows you to use the driver inside the ROS application. It is a ROS node which setup and manage your driver:
- initialize the node
- initialize the driver/library
- keep the driver running while the node is running
- cleanly stops the driver when the node shuts down



### ROS wrapper example

``` python3
#!/usr/bin/env python

import rospy
from my_robot_driver.motor_driver import MotorDriver

from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

class MotorDriverROSWrapper:

    def __init__(self):
        max_speed = rospy.get_param("~max_speed", 8)
        publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        publish_motor_status_frequency = rospy.get_param("~publish_motor_status_frequency", 1.0)
        
        self.motor = MotorDriver(max_speed=max_speed)

        rospy.Subscriber("speed_command", Int32, self.callback_speed_command)
        rospy.Service("stop_motor", Trigger, self.callback_stop)

        self.current_speed_pub = rospy.Publisher("current_speed", Int32, queue_size=10)
        self.motor_status_pub = rospy.Publisher("motor_status", DiagnosticStatus, queue_size=1)

        rospy.Timer(rospy.Duration(1.0/publish_current_speed_frequency), self.publish_current_speed)
        rospy.Timer(rospy.Duration(1.0/publish_motor_status_frequency), self.publish_motor_status)

    def publish_current_speed(self, event=None):
        self.current_speed_pub.publish(self.motor.get_speed())

    def publish_motor_status(self, event=None):
        status = self.motor.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))

        msg = DiagnosticStatus()
        msg.values = data_list

        self.motor_status_pub.publish(msg)

    def stop(self):
        self.motor.stop()

    def callback_speed_command(self, msg):
        self.motor.set_speed(msg.data)
        
    def callback_stop(self, req):
        self.stop()
        return {"success": True, "message": "Motor has been stopped"}

if __name__ == "__main__":
    rospy.init_node("motor_driver")

    motor_driver_wrapper = MotorDriverROSWrapper()
    rospy.on_shutdown(motor_driver_wrapper.stop)

    rospy.loginfo("Motor driver is now started, ready to get commands.")
    rospy.spin()

```


## Organize driver package
If the driver and the wrapper are developed togheter, it is possible to put them in the same package, while in general the driver is indipendent from the wrapper, so they need different packages.

### python driver package

```
my_robot_driver/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts
│   └── motor_ros_wrapper.py
└── src
    └── my_robot_driver
        └── __init__.py
        └── motor_driver.py
```

Where:
* setup.py
    ``` python
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['my_robot_driver'],
        package_dir={'': 'src'}
    )

    setup(**d)
    ```

* CMakeLists.txt
    ```
    catkin_python_setup()
    ```


### c++ driver package

```
my_robot_driver/
├── CMakeLists.txt
├── package.xml
├── include
│   └── my_robot_driver
│       └── motor_driver.hpp
└── src
    └── motor_ros_wrapper.cpp
```

Where:
    ``` python
    ...
    include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

    add_executable(motor_driver src/motor_ros_wrapper)
    target_link_libraries(motor_driver ${catkin_LIBRARIES})
    ```



