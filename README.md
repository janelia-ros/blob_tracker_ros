# blob_tracker_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
rosrun blob_tracker blob_tracker
```

```shell
roslaunch blob_tracker blob_tracker.launch
```

```shell
rosrun image_view image_view image:=/camera/blob_out/image_raw
```

```shell
rostopic echo /camera/blobs
```

```shell
rosrun rqt_reconfigure rqt_reconfigure
```

