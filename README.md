# RobRehabSystem #

Customizable robotic rehabilitation system

## Overview

**RobRehabSystem** is a set of executables (**RobRehabControl** and **RobRehabServer**) intended for control of physical rehabilitation robots. Its generic programming and communication interfaces allow it to integrate with a variety of client applications and custom plugins. 

For more info, refer to [**RobRehabSystem** Wiki](https://github.com/Bitiquinho/RobRehabSystem/wiki).

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
