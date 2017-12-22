See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#iop_ocu_slavelib_fkie).  
List of client service plugins in this repository:

[iop_client_costmap2d_fkie: CostMap2DClient](#iop_client_costmap2d_fkie-costmap2dclient)  
[iop_client_measurement_sensor_fkie: MeasurementSensor](#iop_client_measurement_sensor_fkie-measurementsensorclient)  
[iop_client_path_reporter_fkie: PathReporter](#iop_client_path_reporter_fkie-pathreporterclient)  


## _iop_client_costmap2d_fkie:_ CostMap2DClient

Request map from [CostMap2D](https://github.com/fkie/iop_sensing#iop_costmap2d_fkie-costmap2d) and publish the occupancy grid with tf to ROS.

#### Parameter:

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id.

_tf_frame_costmap (str_, Default: "costmap")

> Defines the map frame id.

_hz (int_ Default: 1.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_costmap (nav_msgs::OccupancyGrid)_

> Map reported from CostMap2D service.

#### Subscriber:

> None

#### TF:

_tf_frame_odom_ -> _tf_frame_costmap_


## _iop_client_measurement_sensor_fkie:_ MeasurementSensorClient

Request measurements from [MeasurementSensor](https://github.com/fkie/iop_sensing#iop_measurement_sensor_fkie-measurementsensor) and publish these as measurement message defined in [iop_msgs_fkie](https://github.com/fkie/iop_core/tree/master/iop_msgs_fkie).

#### Parameter:

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

#### Publisher:

_measurement (iop_msgs_fkie::Measurement)_

> Measurement messages

#### Subscriber:

> None


## _iop_client_path_reporter_fkie:_ PathReporterClient

Request historical and planned paths from [PathReporter](https://github.com/fkie/iop_sensing#iop_path_reporter_fkie-pathreporter) and publish these to ROS as ```nav_msgs::Path``` and ```geographic_msgs::GeoPath```.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame id used in ROS for global coordinates.

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_historical_global_path (nav_msgs::Path)_
_historical_global_geopath (geographic_msgs::GeoPath)_

> Histprical path.

_planned_global_path (nav_msgs::Path)_
_planned_global_geopath (geographic_msgs::GeoPath)_

> Planned path.

#### Subscriber:

> None
