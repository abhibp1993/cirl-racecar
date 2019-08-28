# Racecar MQP
Jason Ashton, Sean Hunt, Myles Spencer

## Setup variables
`source devel/setup.bash`

## To pull submodules
`git submodule init`  
`git submoudle update`

## Dependencies
`sudo apt install ros-kinetic-tf2-bullet ros-kinetic-bfl ros-kinetic-geographic-msgs`

## Compiling on a different computer
When not on the car, you likely don't have CUDA and don't want to run ZED packages  
Instead of `catkin_make`, run `./cm-noncar.sh` which runs catkin_make without zed packages
