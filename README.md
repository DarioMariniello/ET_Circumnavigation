# circumnavigation-planner

A ROS package for target tracking and circumnavigation, for the scenario of moving target.

## Installation

We assume that you have `ROS` and `catkin_tools` installed on your computer.

- Clone this repository and its dependencies in your catkin workspace
```
cd <your-catkin-ws>/src
git clone <path-to-this-repo>
git clone <path-to-geomtwo>
```

- Make and source your catkin workspace
```
cd <your-catkin-ws>
catkin build
source ./devel/setup.bash
```

## Uninstall

- Remove the repository and the dependencies from your catkin workspace.
```
cd <your-catkin-ws>/src
rm <this-repo-folder>
rm <geomtwo-folder>
```
