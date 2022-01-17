# Basic ocnfiguration

## package.xml

* <depend> General ROS package dependency including build_depend, build_export_depend, exec_depend
* <build_depend> ROS package dependency used only for building
* <build_export_depend> ROS package dependency used only for building which exports a header
* <exec_depend> ROS package dependency required only when running the package



## CMakeLists.txt

* find_package(catkin REQUIRED COMPONENTS roscpp rospy)
    Find ROS package dependency. Here are listed the dependencies list in <depend> or <build_depend> of package.xml

* include_directories(include ${catkin_INCLUDE_DIRS})
    Include directories. Before compiling, it is needed to collect all the header paths.

* catkin_package(CATKIN_DEPENDS angles roscpp std_msgs)
    Exporting interfaces. Here we declare the library and header packages needed by all the interfaces you export to other ROS packages. Here are listed the dependencies list in <depend> or <build_export_depend> of package.xml


# C++ System Library Dependencies
When your package depends on a system C++ library, there are usually several kinds of dependencies which must be declared in your package.xml and CMakeLists.txt files.

## package.xml
* <build_depend> Declare packages needed for building your programs, including development files like headers, libraries and configuration files. For each build dependency, specify the corresponding rosdep key. 
* <build_export_depend> Declare packages needed for building your programs and furthermore that exports a header including an imported dependency
* <exec_depend> Declare shared (dynamic) libraries, executables, Python modules, launch scripts and other files required for running your package
* <depend> Combines the previous 3 tags. It is not raccomended for system dependencies.


## CMakeLists.txt

* Finding the library - First, CMake needs to find the library.Most boost C++ components are fully implemented in the header files, then they do not require a separate shared library at run-time.
    find_package(Boost REQUIRED)
    But the boost thread implementation does require a library
    find_package(Boost REQUIRED COMPONENTS thread)

* Include directories
    include_directories(include ${Boost_INCLUDE_DIRS})


# Python Module Dependencies
When your Python package imports other python modules, package.xml should provide a <exec_depend> with the appropriate package name.


## package.xml
* <exec_depend> System dependencies (rosdep name)
* <exec_depend> ROS infrastructure modules (rosdep, rospkg), ROS Python package (rospy, roslaunch)
* <depend> ROS message or service





