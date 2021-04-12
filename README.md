# VISFS - Visual-IMU Sensors Fusion SLAM 
## Requirements

-   C++17 GCC compiler
-	cmake	<http://www.cmake.org>
-	Eigen3	<https://eigen.tuxfaily.org>
-	g2o		<https://github.com/RainerKuemmerle/g2o>
-	ceres	<http://www.ceres-solver.org>
-	ros		<https://www.ros.org>
-	opencv	<https://opencv.org>

On Ubuntu / Debian these dependencies are resolved by installing the
following packages.

-	cmake
-	cmake-qt-gui
-	libeigen3-dev
-	libopencv-dev
-	ros-distribution-libg2o

To fix the conflict with g2o installed by ros, which used by rtabmap.Compile the g2o to a static librarry. And set the following options:

-	BUILD_LGPL_SHARED_LIBS=OFF
-	BUILD_SHARED_LIBS=OFF
-	CMAKE_CXX_FLAGS=-fPIC

## Compilation
Our primary development platform is Ubuntu. We recommend a so-called out of source build which can be achieved by the following command sequence.

### Build VISFS algorithm libarary.
-	`mkdir build`
-	`cd build`
-	`cmake .. -DCMAKE_BUILD_TYPE=Release`
-	`make`

### Build the interface of algorithm.
-	`cd Interface/ROS`
-	`mkdir build`
-	`cmake .. -DROS_BUILD_TYPE=Release`
-	`make`

## Environment setup
	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${YOUR VISFS PATH}/VISFS/Interface/ROS

## Launch
Example for build map:

	roslaunch VISFS simMapping.launch

Example for localization:

	roslaunch VISFS simcar_localization.launch

## Debug

	roslaunch VISFS simVISFS.launch

Or substitute the code with following:
```
<node pkg="VISFS" type="VISFSInterfaceROSNode" name="VISFSInterfaceROSNode" output="screen" launch-prefix="gdb -ex run --args">
```

## Message to teammates
The master is a stable branch.
The develop a branch to be merge new feature or fix bugs associate with issues.

When need to add a new feature to this repo,
1. Create a issues.
2. Work on a new branch assosiate with new issues.
3. Pull request merge to develop.
4. When test stable, merge to master.

## Contact information
-	[Eddy](mailto:417480141@qq.com)

