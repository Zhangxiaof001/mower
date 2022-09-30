# Build
catkin_make

# how to use
1. simulation
source devel/setup.bash
roslaunch open_mower simulation_mower.launch

source devel/setup.bash
bash utils/mower_buttons/press_start.sh

2. robot
source devel/setup.bash
roslaunch open_mower mower.launch

source devel/setup.bash
bash utils/mower_buttons/press_start.sh
