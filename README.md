# Installation

1. Add new packages to your catkin workspace using wstool:
```
cd agrobot/src
wstool merge --merge-replace visualization/install/visualization.rosinstall -y
wstool update -j8
cd ..
```

2. Build visualization:
```
catkin build visualization
```

Remember to source your workspace:
```
source agrobot/devel/setup.bash
```

# Usage

TODO
