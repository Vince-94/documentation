# ROS container
https://registry.hub.docker.com/_/ros/
http://wiki.ros.org/docker/Tutorials/Docker


## Pull a ROS docker image

1. Pull down a ROS container image
    ```
    $ sudo docker pull osrf/ros:noetic-desktop-full
    ```


## Creating a ROS docker image
1. creating the Dockerfile
    ```
    $ cd catkin_ws
    $ code Dockerfile
    ```
2. modify the Dockerfile
    ```
    FROM ros:noetic-desktop-full
    ```



## Using ROS docker
```
$ sudo docker run -it \
                  -v ~/catkin_ws:/root/catkin_ws \
                  -v ~/.ssh:/root/.ssh \
                  --network host \
                  --env ROS_MASTER_URI=http://<your hostname>:11311 \
                  --name [docker_name] \
                  ros:noetic-desktop-full
```




### Open new tabs
Open a new tab
```
docker exec -it [docker_name] bash
```

