dependence
opencv
eigen
g2o
ceres
ros

debug
<node pkg="VISFS" type="VISFSInterfaceROSNode" name="VISFSInterfaceROSNode" output="screen" launch-prefix="gdb -ex run --args">
  
master a stable branch

develop a branch to be merge new feature or fix bugs associate with issues.

When need to add a new feature to this repo,
1. create a issues.
2. work on a new branch assosiate with new issues.
3. pull request merge to develop.
4. When test stable merge to master.


To fix the conflict with g2o installed by ros, which used by rtabmap.
Compile the g2o to a static librarry.
Set the options:
BUILD_LGPL_SHARED_LIBS=OFF
BUILD_SHARED_LIBS=OFF
CMAKE_CXX_FLAGS=-fPIC
