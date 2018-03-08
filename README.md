<p align="center">
  <img src="docs/img/rehab_resized.gif" align="left"/>
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/eesc_resized.gif" align="right"/>
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

## Robot Multi-Level Configuration

In an effort to generalize and parameterize robot control in and efficient way, wherever it makes sense, and facilitate implementation of device specific behaviour, when needed, **RobotSystem-Lite** defines a multi-layer robot configuration scheme.

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/control_abstraction_full.png" width="800"/>
</p>

From top to bottom levels, the control application is expected to have/define:

- A [hash table](https://en.wikipedia.org/wiki/Hash_table) containing one or more generic robot instances, complemented by:
    - A [robot configuration](https://labdin.github.io/RobotSystem-Lite/robot_config.html) JSON file (inside **<root_dir>/config/robots/**), defining its used high-level control implementation and actuators
    - The robot/actuators control itself, implemented as a plug-in library (inside **<root_dir>/plugins/robot_control/**), according to [Robot Control Interface](https://github.com/LabDin/Robot-Control-Interface) description
    - A list of generic actuators list, each one, in turn, complemented by:
        - An [actuator configuration](https://labdin.github.io/RobotSystem-Lite/actuator_config.html) JSON file (inside **<root_dir>/config/actuators/**), defining the lower-level control implementation, sensors and motor configuration and their related control variables (position, force, etc.)
        - A list of generic sensors, each one, in turn, complemented by:
            - [Sensor configuration](docs/sensor_config.html) JSON file (inside **<root_dir>/config/sensors/**), defining the hardware/virtual input device/channel, reference sensor (if needed) and signal processing/conversion options
            - The signal input code itself, implemented as a plug-in library (inside **<root_dir>/plugins/signal_io/**), according to [Signal I/O Interface](https://github.com/LabDin/Signal-IO-Interface) description
        - Generic actuation motor, complemented by:
            - [Motor configuration](docs/motor_config.html) JSON file (inside **<root_dir>/config/motors/**), defining the hardware/virtual output device/channel and signal generation options
            - The signal output code itself, implemented as a plug-in library (inside **<root_dir>/plugins/signal_io/**), according to [Signal I/O Interface](https://github.com/LabDin/Signal-IO-Interface) description

With that structure, a multi-level control process can interact with external clients through a single interface (for comprehending the difference between **joints** and **axes**, see [**Robot Control Interface** rationale](https://github.com/LabDin/Robot-Control-Interface#the-jointaxis-rationale)):

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/control_flux.png" width="800"/>
</p>
            
## Communication Interfaces

**RobotSystem-Lite** client applications communicate with it through [**IP** connections](https://en.wikipedia.org/wiki/Network_socket), with fixed-size **512 bytes** messages. Depending on the type of message sent or received, a specific data format and underlying transport ([TCP](https://pt.wikipedia.org/wiki/Transmission_Control_Protocol) or [UDP][https://pt.wikipedia.org/wiki/User_Datagram_Protocol)) must be used.

<p align="center">
  <img src="https://raw.githubusercontent.com/LabDin/RobotSystem-Lite/master/docs/img/robot_communications.png" width="600"/>
</p>

### Request/Reply messages

Messages requesting state changes or information about the robot are sent by clients occasionally and their arrival should be as guaranteed as possible. Therefore, these messages are transmitted to the server through **TCP** sockets, on port **50000**. Possible messages (and corresponding reply values) are:

- **Robot Info**
- **ROBOT_ENABLE** (0x01): turn on and enable control for the robot of corresponding index
- **ROBOT_DISABLE** (0x02): turn off and disable control for the robot of corresponding index
- **ROBOT_RESET** (0x03): clear errors and calibration values for the robot of corresponding index
- **ROBOT_OPERATE** (0x04): set the robot of corresponding index to normal operation
- **ROBOT_OFFSET** (0x05): set the robot of corresponding index to joints offset measurement
- **ROBOT_CALIBRATE** (0x06): set the robot of corresponding index to joints amplitude measurement
- **ROBOT_PREPROCESS** (0x07): call implementation-specific post-calibration routines for the robot of corresponding index

### Info messages

The same **TCP** connection could be used to send more complex messages, in string format (character array with null terminator). Currently, only one type of info message is supported, but more are planned (like setting user name):

- **Robots info message**: this message is divided in two parts:
  - **Client Request**: Request information about all available robots, [axes and joints](https://bitiquinho.github.io/Robot-Control-Library/joint_axis_rationale.html), performed by sending a single **NULL** (0x00) byte to **RobRehabServer**
  - **Server Reply**: Information returned to requesting client. Serialized string representation in the format specified by currently used [data I/O implementation](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html#a4663a3c54534f571507ed6bdfd9ceb4d). In the default **JSON** format, it would be something like:

```json
{ "id":"robot_name", "axes":[ "r1_x", "r1_y", "r2_theta" ], "joints":[ "r1_0", "r1_1", "r2_0" ] }
```


### Axis update messages

Messages transporting online values for [axes](https://bitiquinho.github.io/Robot-Control-Library/joint_axis_rationale.html) control variables (measurements or setpoints) should arrive as quickly as possible, and there is no advantage in resend lost packets, as their validity is short in time. Thereby, these messages should be transmitted to the server through lower-latency **UDP** sockets, on port **50001**, as byte and [single precision floating-point](https://en.wikipedia.org/wiki/Single-precision_floating-point_format) arrays (to prevent string parsing overhead), with data organized like:

- **Axes measurements** (sent from server to connected clients):

Axes number | Index 1 | Position | Velocity |  Force  | Acceleration | Stiffness | Damping | Index 2 | ...
:---------: | :-----: | :------: | :------: | :-----: | :----------: | :-------: | :-----: | :-----: | :-:
   1 byte   | 1 byte  | 4 bytes  | 4 bytes  | 4 bytes |   4 bytes    |  4 bytes  | 4 bytes | 1 byte  | ...

- **Axes setpoints** (sent from clients to server):

Axes number | Index 1 |  Mask  | Position | Velocity |  Force  |  Acceleration  | Stiffness | Damping | Index 2 | ...
:---------: | :-----: | :----: | :------: | :------: | :-----: | :------------: | :-------: | :-----: | :-----: | :-:
   1 byte   | 1 byte  | 1 byte | 4 bytes  | 4 bytes  | 4 bytes |    4 bytes     |  4 bytes  | 4 bytes | 1 byte  | ...

Where the mask is a [bit field](https://en.wikipedia.org/wiki/Bit_field) used by clients to define which control setpoint variables are the ones they wants to be updated, according to their relative indexes at the array. To set e.g. **position** (index **0**) and **force** (index **2**) for update, the mask value should be:

    mask = ( 1 << 0 ) | ( 1 << 2 )


### Joint update messages

Similarly to axis update messages, messages containing online [joints](https://bitiquinho.github.io/Robot-Control-Library/joint_axis_rationale.html) data could are transmitted as well, over **UDP** sockets on port **50002**:

- **Joint measurements** (sent from server to connected clients):

Joints number | Index 1 | Position |  Force  | Stiffness | Reserved | Index 2 | ...
:-----------: | :-----: | :------: | :-----: | :-------: | :------: | :-----: | :-:
   1 byte     | 1 byte  | 4 bytes  | 4 bytes |  4 bytes  | 20 bytes | 1 byte  | ...

The reserved values are destined to applications that could make use of extra joint-related data, like [**EMG**](https://en.wikipedia.org/wiki/Electromyography) signals.


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

    $ ./RobRehabControl <config_dir> <config_type> <top_config_file>
    
For more info about application usage, refer to [RobRehabControl Wiki Entry](https://github.com/Bitiquinho/RobRehabSystem/wiki/RobRehabControl)
