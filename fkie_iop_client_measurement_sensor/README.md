This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_measurement_sensor:_ MeasurementSensorClient

Request measurements from [MeasurementSensor](https://github.com/fkie/iop_sensing/blob/master/fkie_iop_client_cbrn_sensor/README.md) and publish these as measurement message defined in [fkie_iop_msgs](https://github.com/fkie/iop_msgs/tree/master/fkie_iop_msgs).

#### Parameter:

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

#### Publisher:

_measurement (fkie_iop_msgs::Measurement)_

> Measurement messages

#### Subscriber:

> None
