# pcl2pcd

pcl2pcd provides ros services to save pointcloud msgs to pcd files

## ROS_API

### pcl2pcd_node

#### Service

`save_to_pcd` (pcl2pcd_node/SaveToPcd)

When the node receives an `save_to_pcd` request, it records the timestamp this request was sent and it will call `assemble_scans2` service from `laser_assembler` with this time as the ending time and the previous call's timestamp as the starting time for assembling laser scan to point cloud.

If this is the first request to `save_to_pcd`, it will simple get all the point in `laser_assembler` up till this time.

#### Argument

`assembler` (string, default `assembler`):

The namespace of laser assembler

`use_sim_time` (bool, default `true`):

Use simulation time when play a bag file

## Usage

Launch the `pcl2pcd_node` under the namespace of `laser_assembler`.

If you want to save all the points up till now to a file named `test_pcd.pcd`, simply do:

```
rosservice call /assembler/save_to_pcd /home/user/Desktop/test_pcd.pcd
```
