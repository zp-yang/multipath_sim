# multipath_sim
GPS multipath simulation in Gazebo

Install:
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:zp-yang/multipath_sim
cd ../
catkin init
catkin build
. devel/setup.bash
roslaunch multipath_sim test.launch
```

Make sure to put this repo in the src/ directory of a catkin workspace

catkin_ws/src/multipath_sim/
