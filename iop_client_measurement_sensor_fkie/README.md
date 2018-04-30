This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_measurement_sensor_fkie:_ MeasurementSensorClient

Request measurements from [MeasurementSensor](https://github.com/fkie/iop_sensing/blob/master/iop_measurement_sensor_fkie/README.md) and publish these as measurement message defined in [iop_msgs_fkie](https://github.com/fkie/iop_msgs/tree/master/iop_msgs_fkie).

#### Parameter:

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

#### Publisher:

_measurement (iop_msgs_fkie::Measurement)_

> Measurement messages

#### Subscriber:

> None
