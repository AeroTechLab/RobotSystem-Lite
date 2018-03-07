**RobRehabSystem** client applications must communicate to it through [[RobRehabServer|RobRehabServer]] **IP** connections, with fixed-size **512 bytes** messages. Depending on the type of message sent or received, a specific data format and underlying protocol should be used.

### Command/Request messages

Messages requesting state changes or information about one or more robot are sent by clients occasionally and their arrival should be as guaranteed as possible. Therefore, these messages should be transmitted to the server through **TCP** sockets, on port **50000**, as a byte array with data organized like:
 
Number of commands | Robot index 1 | Command ID 1 | Robot index 2 | Command ID 2 | ...
:----------------: | :-----------: | :----------: | :-----------: | :----------: | :-:
1 byte             |    1 byte     |    1 byte    |    1 byte     |    1 byte    | ... 

Currently available command identifiers, their values and roles are the following:

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
{ "robots":[ "robot_1", "robot_2" ], "axes":[ "r1_x", "r1_y", "r2_theta" ], "joints":[ "r1_0", "r1_1", "r2_0" ] }
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
    1 byte    | 1 byte  | 4 bytes  | 4 bytes |  4 bytes  | 20 bytes | 1 byte  | ...

The reserved values are destined to applications that could make use of extra joint-related data, like [**EMG**](https://en.wikipedia.org/wiki/Electromyography) signals.