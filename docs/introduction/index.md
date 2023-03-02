# Introduction to ROS

Welcome to the ROS FE workshop. In this part of the workshop you will learn the fundamentals of ROS.

## Why ROS?

ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. 

ROS has three levels of concepts: the Filesystem level, the Computation Graph level, and the Community level.

### ROS Filesystem Level

The basic unit for software code organization in ROS is a package. It can contain code for processes, liberaries, datasets, configuration files and custom communication types definitions. Packages can be sensibly grouped into metapackages. Each package contains a package manifest (package.xml) that provides the metadata about the package.

### ROS Computation Graph Level

In ROS, processes run as nodes in a peer-to-peer network processing data together. This network is called ROS Computation Graph and it is supported by the name registration and lookup services of the ROS Master, by the parameter storing and serving services of the Parameter Server, and by the ROS logging mechanism. ROS nodes communicate by excanging data using messages in communication models called topics, services and actions.

### ROS Community Level

There is a large number of ROS developers and robotic systems developers using ROS worldwide. Separate communities can exchange software and knowledge using ROS distributions, code repositories, the ROS Wiki and ROS Answers forums, blog and mailing lists. To get the participants of this course acquainted with the availabe resources, reading through some of the ROS Wiki topics will be encouraged by providing links to the ROS Wiki contents.