In order to keep consistency across user developed **control plug-ins** (for [single actuators](https://bitiquinho.github.io/Robot-Control-Library/classACTUATOR__CONTROL__INTERFACE.html) or [whole robots](https://bitiquinho.github.io/Robot-Control-Library/classROBOT__CONTROL__INTERFACE.html)), and allow easier interoperation between them, we propose the following conventions for each dimension variable used by implementations as input and output:  

Control Variabe | Translational Unit     | Rotational Unit
:-------------: | :--------------------: | :-------------------------------:
Position        | meter (m)              | radian (rad)
Velocity        | m / second (m/s)       | rad / second (rad/s)
Force/Torque    | Newton (N)             | Newton x meter (Nxm)
Acceleration    | meter / second² (m/s²) | radian / second² (rad/s²)
Stiffness       | Newton / meter (N/m)   | Newton x meter / radian (Nxm/rad)
Damping         | N x s / m              | N x m x s / rad


Also, **measurements** should be acquired from **sensors** and **setpoints** should be provided to **motors** in the same way. Scale conversions between that and specifc units used by each [signal I/O plugin](https://bitiquinho.github.io/Robot-Control-Library/classSIGNAL__IO__INTERFACE.html) implementation could be defined in [input](https://bitiquinho.github.io/Robot-Control-Library/sensor_config.html) and [output](https://bitiquinho.github.io/Robot-Control-Library/motor_config.html) gains configuration.