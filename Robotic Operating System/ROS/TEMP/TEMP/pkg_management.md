# Packages management

## roslocate
It retrieves version-control information about a ROS package, for source-control repositories.

roslocate
	info [pkg]		    info about the version-control system of the package
	describe [pkg]		description of the repository under version-control system


## rosws
It manipulates the catkin workspace.

### Initializing workspace
rosws init [target_path] [source_path]

The initialization:
1.	looks for .rosinstall yaml file inside the repository (folder, file, uri)
2.	creates new .rosinstall file at the target path
3.	generates ROS setup files

### Setting workspace
Adding/changing manually configuration in the .rosinstall file.

rosws set [pkg_name] [option] [--version=X]

### Merging workspace
It merges workspace configurations automatically with other .rosinstall files.

rosws merge [option]
	-a			merging by deleting given entry and appending new one
	-k			(default) merging by keeping existing entry and discarding new one
	-r			merging by replacing given entry with new one maintaining ordering
	-t [workspace_name]	target workspace to use

### Updating workspace
It updates or checks out some of your config elements, in the case the uri has changed.

rosws update -t ~/catkin_ws
rosws update [pkg_name]*



## Scenarios

* Creating workspace
cd ~/catkin_ws
rosws init /opt/ros/$ROS_DISTRO -c


* Adding manually new a repository to .rosinstall
roslocate info [repo_name]
cd ~/catkin_ws
rosws set [repo_name] --git [uri]
rosws update 


* Merging  automatically new repository to .rosinstall
roslocate info [repo_name] | rosws merge -
rosws update



-----------------------------------------------------------


## Generate a list of dependancies

* Unmatch dependanciies of all the workspace (not installed)
    ```
    rosdep install -si --reinstall --from-path ~/catkin_ws/src
    ```
