FROM ros:kinetic-ros-base-xenial

ENV UTCOUPE_WORKSPACE /utcoupe/coupe20
RUN mkdir -p $UTCOUPE_WORKSPACE/ros_ws/src
ENV ROS_LANG_DISABLE=genlisp:geneus

RUN apt-get update
RUN apt-get install wget libsfml-dev libarmadillo-dev sudo -qq -y
RUN apt-get install ros-kinetic-tf2 ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-diagnostic-updater ros-kinetic-roslint ros-kinetic-camera-info-manager ros-kinetic-rosserial-arduino ros-kinetic-rosbridge-suite ros-kinetic-tf2-web-republisher ros-kinetic-serial ros-kinetic-dynamixel-sdk ros-kinetic-rosserial-python ros-kinetic-tf2-geometry-msgs ros-kinetic-urg-c ros-kinetic-urg-node -y -qq
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p /temp/utcoupe_install
WORKDIR /temp/utcoupe_install
RUN wget https://github.com/utcoupe/coupe20/raw/master/scripts/install_external_nodes.sh && chmod +x install_external_nodes.sh
RUN bash -c "source /opt/ros/kinetic/setup.sh && ./install_external_nodes.sh"
RUN rm -rf /tmp/utcoupe*

WORKDIR $UTCOUPE_WORKSPACE/ros_ws
# Tests if the setup works
RUN bash -c "source /opt/ros/kinetic/setup.sh && catkin_make"
RUN rm -rf build devel


WORKDIR $UTCOUPE_WORKSPACE/ros_ws
CMD echo "Using $UTCOUPE_WORKSPACE as ROS workspace."

