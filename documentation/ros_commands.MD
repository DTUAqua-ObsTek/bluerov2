# Basic Commandline scripting in ROS #

## Quick Overview of some Graph Concepts in ROS ##

**Nodes:** A node is an executable that uses ROS to communicate with other nodes.

**Messages:** ROS data type used when subscribing or publishing to a topic.

**Topics:** Nodes can publish messages to a topic as well as subscribe to a topic to receive messages. 

**ROS Master:** It is the primary machine to which all nodes are connected, and helps nodes find each other at startup for p2p communication.

ROS client libraries allow nodes written in different programming languages to communicate 

**rospy**: Python Client Library

**roscpp**: C++ Client Library

If running ROS on your own PC, roscore is the first thing you should run when using ROS

`roscore`

Now that the ROS Master is up and running, you can run a bunch of other commandlines-

rosrun allows you to use the package name to directly run a node within a package (without having to know the package path). 

`rosrun [package_name] [node_name]`

fx `rosrun rviz rviz`

If you want to see the list of topics 
`rostopic list`

If you want to see the list of nodes
`rosnode list`

If you want to see the information being published on a topic
`rostopic echo [topic]`

In general, you can use the following to see the entire set of possible arguments

`rostopic -h`
or
`rosnode -h`

You can also visualise the connection graph using 
`rosrun rqt_graph rqt_graph`

The data being published on topics can also be plotted against time.
`rosrun rqt_plot rqt_plot`
