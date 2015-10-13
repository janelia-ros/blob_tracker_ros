#blob_tracker_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
ROS_NAMESPACE=camera rosrun blob_tracker blob_tracker
```

```shell
ROS_NAMESPACE=camera roslaunch blob_tracker blob_tracker.launch manager:=camera_nodelet_manager
```

```shell
rosrun image_view image_view image:=/camera/blob_out/image_raw
```

```shell
rostopic echo /camera/blobs
```

```shell
rostopic pub -1 /camera/blob_tracker/save_background_image std_msgs/Empty
```

