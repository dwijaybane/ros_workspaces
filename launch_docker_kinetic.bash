#!/bin/bash 

# Requires
#   docker
#   nvidia-docker2
#   an X server

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Image Name From Dockerhub Preferably 
IMAGE=${1:-streaminterrupt/ros-lab:kinetic-v2}

#custom port
# PORT=4545
# PORT_R=8787

# If we are called without a parameter to specify a new image, 
# let's make sure we are on the latest image
if [ $# -eq 0 ]; then
    echo "Will check for the latest image on the docker hub."
    # docker pull $IMAGE
fi

xhost +local:root
GPU=0

#Launch Container 'ros_lab1'
# Below Two lines are for sound card detection. This will mess up host sound.
# --device=/dev/snd:/dev/snd:<rwm> : This gives access to specific device with read, write or mknod
# --privileged : This mode gives access to all devices
# For RGB-D Sensor
# --device=/dev/:/dev/:m \
# --privileged
nvidia-docker run --rm -it --init \
    --name ros_lab2 \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $PWD:/notebooks -w /notebooks \
    $IMAGE


xhost -local:root
