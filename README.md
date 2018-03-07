<p align="center">
  <img src="https://raw.githubusercontent.com/Bitiquinho/RobRehabSystem/master/img/rehab_resized.png" align="left"/>
  <img src="https://raw.githubusercontent.com/Bitiquinho/RobRehabSystem/master/img/eesc_resized.png" align="right"/>
</p>
  

***
***

<h1 align="center">RobotSystem-Lite</h1>
  
***
***
  
  


## Overview

**RobotSystem-Lite** is a lightweight customizable robotic control application, with few third-party dependencies, for easier deployment in embedded systems. Its generic programming and communication interfaces allows it to integrate with a variety of client applications (for remote comunication) and custom plugins (for implementing specific control algorithms and robotic hardware I/O support). 

<p align="center">
  <img src="https://raw.githubusercontent.com/Bitiquinho/RobRehabSystem/master/docs/img/top_level_system-basic.png" width="800"/>
</p>
<p align="center">
  (High-level RobotSystem-Lite interfacing examples)
</p>

Its original goal is to enable combination of robotic rehabilitation with alternative approaches for physical therapy, like remote assistance and multiplayer "serious" video-games.

<p align="center">
  <img src="https://raw.githubusercontent.com/Bitiquinho/RobRehabSystem/master/docs/img/top_level_system-full.png" width="800"/>
</p>
<p align="center">
  (One of proposed RobotSystem-Lite usages)
</p>


However, its configuration flexibility is intended for allowing other usages of robotic control besides rehabilitation.

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
