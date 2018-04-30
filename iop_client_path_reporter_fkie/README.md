This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_path_reporter_fkie:_ PathReporterClient

Request historical and planned paths from [PathReporter](https://github.com/fkie/iop_sensing/blob/master/iop_path_reporter_fkie/README.md) and publish these to ROS as ```nav_msgs::Path``` and ```geographic_msgs::GeoPath```.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame id used in ROS for global coordinates.

_tf_frame_odom (str_, Default: "odom")

> TF frame id used to publish the local coordinates.

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_historical_global_path (nav_msgs::Path)_
_historical_global_geopath (geographic_msgs::GeoPath)_

> Histprical path.

_planned_global_path (nav_msgs::Path)_
_planned_global_geopath (geographic_msgs::GeoPath)_
_planned_local_path (nav_msgs::Path)_
> Planned path.

#### Subscriber:

> None
