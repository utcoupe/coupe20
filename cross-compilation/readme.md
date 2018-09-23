# Cross-compilation using docker
The goal is to use fast computer to compile our project on slow computer platforms (raspberry pi in our case). The docker image contains the basic setup to compile the project. It may work on Apple MacOS and Microsoft Windows (not tested).

For example, on a raspberry pi 3, it makes in average 12 minutes to compile on j1 (j2 doesn't work due to RAM limitation). On a laptop (i7-4710HQ), using this technic, it makes 5 minutes in average using j8.

The main downside is, because x86 doesn't natively support armhf instructions, the arm assembly instructions must be translated to x86 instructions, so why we need the software `qemu`. As a result, compilations are slower than usual on the x86 PC.

## Requirements:
- Docker installed and configured properly. See [installation instruction for Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce).
- For arm compilation on a x86 system: the apt packages `qemu-user-static` and `binfmt-support`.
- The folder `$UTCOUPE_WORKSPACE` must contains all submodules and unpacked libs required by the ros packages.

TODO Make install script for ubuntu.

## How to use
### V1
The script will build the `Dockerfile` inside the selected folder.
Then, it launches the container with a binding to your `$UTCOUPE_WORKSPACE` folder, expecting the image to launch automaticly the compilation using `catkin_make install`. The generated files will be located in the folder `cross_compilation/generated_install/the_selected_arch`. If you have important files in it, please make sure you backup it before launching the script because it will erase it.

The script will create a tgz file containing this folder. You can then send it to the robot by using scp, for example if you chose `armv7`:
```
scp path/to/armv7.tgz user@robotname:~/compiled_binairies
```
and then in ssh (`user@robotname:~/compiled_binairies`)
```
tar -xzf armv7.tgz
```

In order to use the folder as a ROS environnement you need to do the folowing steps in all your remote terminals (or copy/paste this in the remote .bashrc):
```
export _CATKIN_SETUP_DIR=/absolute/path/to/armv7
source path/to/armv7/setup.sh
```
At this point your ready to use it!

### V2
The script will download the last docker image if availlable (not planned yet).
