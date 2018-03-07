**RobRehabControl** is the application responsible for effectively interacting with rehabilitation robots. It leverages [Bitiquinho's Robot Control Library](https://github.com/Bitiquinho/Robot-Control-Library), writing acquired robot measurements to and reading control setpoints from memory buffers, shared with [[RobRehabServer|RobRehabServer]] for interfacing with external applications.

### Command-line arguments

For running it, use the following command, from root project directory:

    $ ./RobRehabControl <config_dir> <config_type> <top_config_file>
    
- **config_dir** is the root configuration path (absolute or relative to the executable), where the top-level configuration file and the robot configuration subdirectories are located. The definition of each robot is done according to [Bitiquinho's Robot Control Library configuration scheme](https://bitiquinho.github.io/Robot-Control-Library/configuration_levels.html) and uses the following subdirectories (paths relative to the executable):  

  - Robot-level: **config_dir/robots** for configuration files and **plugins/robot_control** for robot control implementations
  - Actuator-level: **config_dir/actuators** for configuration files and **plugins/actuator_control** for actuator control implementations
  - Sensor-level: **config_dir/sensors** for configuration files and **plugins/signal_io** for signal input/output implementations
  - Motor-level: **config_dir/motors** for configuration files and **plugins/signal_io** for signal input/output implementations
  - Curve data: **config_dir/curves** for configuration files


- **config_type** is the name (without extension) of [data I/O plugin](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html) library (stored at **plugins/data_io**) implemented for configuration loading (like the default **JSON** one) 


- **top_config_file** is the top-level configuration file (at **config_dir**) name, that contains a **"robots"** list of all robot-level configuration files names (without extension) used for current operation. It's internal format, as all other configuration, should follow the currently used data I/O implementation. In the case of **JSON** plugin, it should be something like:

```json
{ "robots":[ "robot_1_config", "robot_2_config" ] }
```