This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_costmap2d:_ CostMap2DClient

Request map from [CostMap2D](https://github.com/fkie/iop_sensing/blob/master/fkie_iop_costmap2d/README.md) and publish the occupancy grid with tf to ROS.

#### Parameter:

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id.

_tf_frame_costmap (str_, Default: "costmap")

> Defines the map frame id.

_send_inverse_trafo (bool_, Default: false)

> Sets the transform direction while publish TF frame. True: tf_frame_odom -> tf_frame_costmap.

_hz (int_ Default: 1.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_costmap (nav_msgs::OccupancyGrid)_

> Map reported from CostMap2D service.

#### Subscriber:

> None

#### TF:

_tf_frame_costmap_ -> _tf_frame_odom_

