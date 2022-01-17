# Topics

## Publishing rate

It is importnt to separate the reading, processing, and publishing of the data.


### Simple class

``` python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class TemperatureSensor:

    def read_temperature_sensor_data(self):
        # Here you read the data from your sensor
        # And you return the real value
        self.temperature = 30.0

    def __init__(self):
        # Create a ROS publisher
        self.temperature_publisher = rospy.Publisher("/temperature", Float64, queue_size=1)

        # Initialize temperature data
        self.temperature = 0

    def publish_temperature(self):
        msg = Float64()
        msg.data = self.temperature
        self.temperature_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    ts = TemperatureSensor()

    # Create a rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ts.read_temperature_sensor_data()
        ts.publish_temperature()
        rate.sleep()
```


### Read data and publish asynchronously

A ROS Timer allows you to set a callback that will be triggered at a given rate. There are 2 timers: one Timer to read the temperature data from the sensor, and another one to publish this data.

The callback function ha the additial optional parameter "event" which gives time info about timer if needed. 


``` python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class TemperatureSensor:

    def read_temperature_sensor_data(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        self.temperature = 30.0

    def __init__(self):
        # Create a ROS publisher
        self.temperature_publisher = rospy.Publisher("/temperature", Float64, queue_size=1)

        # Initialize temperature data
        self.temperature = 0

    def publish_temperature(self, event=None):
        msg = Float64()
        msg.data = self.temperature
        self.temperature_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    ts = TemperatureSensor()

    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(1.0/10.0), ts.read_temperature_sensor_data)

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(1.0/10.0), ts.publish_temperature)

    # Don't forget this or else the program will exit
    rospy.spin()
```


### Oversampling and averaging

Oversampling is when the publishing rate is higher than the subscribing rate. With more data samples between each publication on the ROS topic, you also have the possibility to filter the data (ex: with a complementary filter or a Kalman filter).

In this example, we’ll simply add a filter that updates the temperature with the average of the 2 last read values.

``` python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class TemperatureSensor:

    def read_temperature_sensor_data(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        if self.temp_index < self.temp_max_index:
            self.temp_data.append(30.0)
            self.temp_index += 1
        else:
            self.temperature = sum(self.temp_data) / len(self.temp_data)
            self.temp_index = 0
            self.temp_data = []

    def __init__(self):
        # Create a ROS publisher
        self.temperature_publisher = rospy.Publisher("/temperature", Float64, queue_size=1)

        # Initialize temperature data
        self.temperature = 0

        self.temp_data = []
        self.temp_index = 0
        self.temp_max_index = 1

    def publish_temperature(self, event=None):
        msg = Float64()
        msg.data = self.temperature
        self.temperature_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    ts = TemperatureSensor()

    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(1.0/100.0), ts.read_temperature_sensor_data)

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(1.0/10.0), ts.publish_temperature)

    # Don't forget this or else the program will exit
    rospy.spin()
```





# Actions

https://docs.ros.org/api/actionlib/html/namespaceactionlib.html

## Action client
``` python
#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from chores.msg import DoDishesAction, DoDishesGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

```


## Action server
``` python
#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from chores.msg import DoDishesAction

class DoDishesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = DoDishesServer()
  rospy.spin()

```


## Tutorials
https://www.youtube.com/watch?v=7kjJ3eJrZKo&list=PLK0b4e05LnzZu5XWBqCKg9IvnIJU0votI&index=33
https://www.youtube.com/watch?v=SZ33EJ8lPD4&list=PLK0b4e05LnzZu5XWBqCKg9IvnIJU0votI&index=35





