dependence
opencv
eigen
g2o
ros

remap input
left/image
right/image
left/info
right/info
wheel_odom

output
odom
odom_info


rosparam
left/image_transport
right/image_transport
approx_sync
queue_size
base_line
camera_frame_id
robot_frame_id
publish_tf

algorithm param
Parameters.h

debug
<node pkg="VISFS" type="VISFSInterfaceROSNode" name="VISFSInterfaceROSNode" output="screen" launch-prefix="gdb -ex run --args">