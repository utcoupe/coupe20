Source code of UTCoupe 2020
=======
|master|devel|
|------|-----|
| [![Build Status](https://travis-ci.org/utcoupe/coupe20.svg?branch=master)](https://travis-ci.org/utcoupe/coupe20) | [![Build Status](https://travis-ci.org/utcoupe/coupe20.svg?branch=devel)](https://travis-ci.org/utcoupe/coupe20) |

# Configuration

First clone the repo to your PC :
```
git clone git@github.com:utcoupe/coupe20.git
```

### Development environment configuration

An automated installation script is available. Go in the 'coupe20' folder and launch :
```
./scripts/install_utcoupe_setup.sh
```

If it is your first installation, answer "y" to all questions.

### Compile the system

The whole system relies on ROS (http://www.ros.org/). Therefore you need to compile the system after having fetched it :
```
cd coupe20/ros_ws
catkin_make
```

### Source the system

Do not forget to source it !
If you are using bash : 
'''
source devel/setup.bash
'''

If you are using zsh : 
'''
source devel/setup.zsh
'''

You can add the source command with the absolute path to setup.bash in ~/.bashrc (or ~/.zshrc) to automate this process.

# Project guidelines

Here are some rules we follow to keep project consistency :

### Git

- Create git branches of format `namespace/package` for ROS packages (e.g. `ai/scheduler`, `memory/map`, etc).

### ROS packages

- Create ROS packages of format `namespace_package` (for alphabetic order sorting purposes)

- Create `topics`/`services`/`actions` in the format `/namespace/package/server_name` if they can be accessed by extern packages (WARNING : with `/` at the beginning to create an absolute name), `server_name` if they are only intern.

- Name definition files `.msg`/`.srv`/`.action` in PascalCase (e.g. `GetValues.srv`) and variables inside in lower case(format `var_name`).

### Python

- PEP8 : 4 indentation spaces (no tabs).

### Data

- Distance units in m, stored as `float32`.

- When describing a shape (circle, line, rectangle, point...), give the position relative to the center of the shape unless it is absolutely necessary.  For example, do not give the corner of a rectangle.

# Webclient

To install the webclient dependency :
```
cd webclient
npm install --only=prod
```

To launch the webclient :
```
npm start
```

Make sure the ROS node `rosbridge_server` starts right.

The webclient can be launched from the robot or a PC connected to the embedded computer (e.g. Raspberry or Lattepanda).

If the server is launched on the robot, go to `http://<ip_of_embedded_computer>:8080`.

Else, go to [http://localhost:8080](http://localhost:8080) and check if the client connects to the robot IP in the parameters.