my_pcl_tutorial
===============

1. Create a catkin package
catkin_create_pkg my_pcl_tutorial pcl_conversions pcl_ros roscpp sensor_msgs

2. Modify the package.xml
<build_depend>libpcl-all-dev</build_depend>
<run_depend>libpcl-all</run_depend>

3. Create a node (my_pcl_tutorial_node.cpp)

4. Modify the CMakeLists.txt
add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

Note: git setup
===============
git config --global user.name "your github username"
git config --get user.name
> your github username

git config --global user.email "your@github.email"
git config --get user.email
> your@github.email
