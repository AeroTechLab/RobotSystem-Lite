# Robot Multi-Level Configuration #         {#configuration_levels}

In an effort to generalize and parameterize robot control in and efficient way, wherever it makes sense, and facilitate implementation of device specific behaviour, when needed, the **Robot Control Library** defines a multi-layer robot configuration scheme.

<p align="center">
  <img src="https://raw.githubusercontent.com/Bitiquinho/Robot-Control-Library/gh-pages/img/control_abstraction_full.png" width="800"/>
</p>

From top to bottom levels, the control application is expected to have/define:

- A [hash table](https://en.wikipedia.org/wiki/Hash_table) containing one or more generic robot instances, complemented by:
    - A [robot configuration](robot_config.html) file/data object, defining its used high-level control implementation and actuators
    - The robot control itself, implemented as a plug-in library, according to ROBOT_CONTROL_INTERFACE description
    - A list of generic actuators list, each one, in turn, complemented by:
        - An [actuator configuration](actuator_config.html) file/data object, defining the lower-level control implementation, sensors and motor configuration and their related control variables (position, force, etc.)
        - The actuator control itself, implemented as a plug-in library, according to ACTUATOR_CONTROL_INTERFACE description
        - A list of generic sensors, each one, in turn, complemented by:
            - [Sensor configuration](sensor_config.html) file/data object, defining the hardware/virtual input device/channel, reference sensor (if needed) and signal processing/conversion options
            - The signal input code itself, implemented as a plug-in library, according to SIGNAL_IO_INTERFACE description
        - Generic actuation motor, complemented by:
            - [Motor configuration](motor_config.html) file/data object, defining the hardware/virtual output device/channel and signal generation options
            - The signal output code itself, implemented as a plug-in library, according to SIGNAL_IO_INTERFACE description
