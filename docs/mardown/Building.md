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