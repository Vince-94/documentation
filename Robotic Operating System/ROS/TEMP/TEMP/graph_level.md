

# Graph resource names
Every graph resource (nodes, topics, services, parameters) is identified by a short string called a graph resource name.

## Global names: /global_name
Global names have a global scope



## Relative names: relative_name
Relative names allow ROS to supply a default namespace. This kind of flexibility can make the organization of a system more clear and, more importantly, can prevent name collisions when groups of nodes from different sources are combined.
They cannot be matched to specific graph resources unless we know the default namespace that ROS is using to resolve them:
- Default namespace = /global_name
- Relative name = relative_name
- Global name = /global_name/relative_name

The default namespace has to be set for each node ("ns" attribute).



## Private names: ~private_name
Private names use the name of their node as a namespace. Private names are often used for parameters and services that govern the operation of a node (for things that are related only to that node, and are not interesting to anyone else). Graph resources referred to by private names remain accessible, via their global names, to any node that knows their name.
- Node global name = /global_name/path
- Relative name = ∼private_name
- Global name = /global_name/path/private_name





# Parameter server
A parameter server is a shared, multi-variate dictionary implemented using XMLRPC. Nodes can use this server to store and retrieve parameters runtime, for static, non-binary data.
The parameter names follow the ROS naming convention as for topics and services.


Way to handle parameters:
* launch
* yaml file

Supported types:
* strings
* 32-bit integers
* doubles
* float
* booleans
* lists of elements
* dictionary
* base64-encoded binary data
* iso8601 dates


### Links
https://www.youtube.com/watch?v=m9FjgacE2OY&list=PLK0b4e05LnzZu5XWBqCKg9IvnIJU0votI&index=53
https://www.youtube.com/watch?v=M_Vr0zJbih4&list=PLK0b4e05LnzZu5XWBqCKg9IvnIJU0votI&index=54


## YAML format

/pkg
    /config
        my_params.yaml
    ...

Type formatting:

``` yaml
text: "Hello"
number_int: 42 
number_float: 21.3
enable_boolean: true
list_of_elements:
    - 1
    - 2
    - 3
    - 4
dictionary: {
    another_text: "World",
    another_number: 12,
}
```

### Loading parameters from a yaml file

* CLI

```
$ rosparam load my_params.yaml
```

* Launch file

``` xml
<launch>
    <rosparam file="$(find my_custom_package)/config/my_params.yaml" />
</launch>
```


### Using parameters in code

* rospy (get parameter)

``` python
x_int = rospy.get_param("/number_int")
```

To check if a parameter exists before accessing it
``` python
if rospy.has_param('/my_integer'):
    rospy.get_param('/my_integer')
```

If the parameter does not exists it's possible to use a default value:
``` python
str_var = rospy.get_param('/my_string', 'this is a default value')
```

* rospy (set parameter)
``` python
rospy.set_param('/another_integer', 12)
```



# Dynamic reconfigure
The dynamic reconfigure is a package providing a means to update parameters at runtime without having to restart the node.

## Package structure
```
/catkin_ws
    /src
        /pkg_name
            /cfg
                dynamic.cfg
```

## Data structure
``` python
#!/usr/env/bin python3
PACKAGE = “pkg_name”
from dynamic_reconfigure.parameter_generator_catkin import *

param = ParametereGenerator()

param.add(‘param_name’, param_type, 0, “description”, init_value, final_value)

exit(gen.generate(PACKAGE, “pkg_name”, “param_file_name”))
```


## CMakeLists.txt
```
# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/dynamic.cfg
)
```

## Link
http://wiki.ros.org/dynamic_reconfigure?distro=melodic






