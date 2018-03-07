<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/img/rehab_resized.png" align="left"/>
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/img/eesc_resized.png" align="right"/>
</p>
  

***
***

<h1 align="center">RobotSystem-Lite</h1>
  
***
***
  
  


## Overview

**RobotSystem-Lite** is a lightweight customizable robotic control application, with few third-party dependencies, for easier deployment in embedded systems. Its generic programming and communication interfaces allows it to integrate with a variety of client applications (for remote comunication) and custom plugins (for implementing specific control algorithms and robotic hardware I/O support). 

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/top_level_system-basic.png" width="800"/>
</p>
<p align="center">
  (High-level RobotSystem-Lite interfacing examples)
</p>

Its original goal is to enable combination of robotic rehabilitation with alternative approaches for physical therapy, like remote assistance and multiplayer "serious" video-games.

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/top_level_system-full.png" width="800"/>
</p>
<p align="center">
  (One of proposed RobotSystem-Lite usages)
</p>


However, its configuration flexibility is intended for allowing other usages of robotic control besides rehabilitation.

**RobRehabControl** is the application responsible for effectively interacting with rehabilitation robots. It leverages [Bitiquinho's Robot Control Library](https://github.com/Bitiquinho/Robot-Control-Library), writing acquired robot measurements to and reading control setpoints from memory buffers, shared with [[RobRehabServer|RobRehabServer]] for interfacing with external applications.

## Robot Multi-Level Configuration

In an effort to generalize and parameterize robot control in and efficient way, wherever it makes sense, and facilitate implementation of device specific behaviour, when needed, the **Robot Control Library** defines a multi-layer robot configuration scheme.

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/control_abstraction_full.png" width="800"/>
</p>

From top to bottom levels, the control application is expected to have/define:

- A [hash table](https://en.wikipedia.org/wiki/Hash_table) containing one or more generic robot instances, complemented by:
    - A [robot configuration](robot_config.html) file/data object, defining its used high-level control implementation and actuators
    - The robot/actuators control itself, implemented as a plug-in library, according to ROBOT_CONTROL_INTERFACE description
    - A list of generic actuators list, each one, in turn, complemented by:
        - An [actuator configuration](actuator_config.html) file/data object, defining the lower-level control implementation, sensors and motor configuration and their related control variables (position, force, etc.)
        - A list of generic sensors, each one, in turn, complemented by:
            - [Sensor configuration](sensor_config.html) file/data object, defining the hardware/virtual input device/channel, reference sensor (if needed) and signal processing/conversion options
            - The signal input code itself, implemented as a plug-in library, according to SIGNAL_IO_INTERFACE description
        - Generic actuation motor, complemented by:
            - [Motor configuration](motor_config.html) file/data object, defining the hardware/virtual output device/channel and signal generation options
            - The signal output code itself, implemented as a plug-in library, according to SIGNAL_IO_INTERFACE description

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

## Building

On a terminal, get the [GitHub code repository](https://github.com/Bitiquinho/RobRehabSystem) with:

    $ git clone https://github.com/Bitiquinho/RobRehabSystem <my_system_folder>

Besides operating system's libraries, this system is dependent on [Bitiquinho's Robot Control Library](https://github.com/Bitiquinho/Robot-Control-Library) and [Bitiquinho's Platform Utils](https://github.com/Bitiquinho/Platform-Utils), that are automatically linked as a [git submodules](https://chrisjean.com/git-submodules-adding-using-removing-and-updating/)

To add those repositories to your sources, navigate to the root project folder and clone them with:

    $ cd <my_system_folder>
    $ git submodule init
    $ git submodule update

With dependencies set, you can now build the system executables (**RobRehabControl** and **RobRehabServer**) to a separate build directory with [CMake](https://cmake.org/):

    $ mkdir build && cd build
    $ cmake .. 
    $ make
    
## Running

**RobRehabControl** and **RobRehabServer** could be executed separated or together, in any order, as the shared buffers for data exchange between them are automatically created by the first process and accessed by the second.

### RobRehabControl command-line arguments

    $ ./RobRehabControl <config_dir> <config_type> <top_config_file>
    
For more info about application usage, refer to [RobRehabControl Wiki Entry](https://github.com/Bitiquinho/RobRehabSystem/wiki/RobRehabControl)

### RobRehabServer command-line arguments

    $ ./RobRehabServer <multicast_ip_address>
    
For more info about application usage, refer to [RobRehabServer Wiki Entry](https://github.com/Bitiquinho/RobRehabSystem/wiki/RobRehabServer)
