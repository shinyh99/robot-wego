# wego

## Cartographer
- made by Google
- https://google-cartographer-ros.readthedocs.io/en/latest/

## Anckerman steering

## How to run robot

### scout boot
`rosrun scout_bringup bringup_can2usb.bash`
### scout bring up
`roslaunch team_one scout.launch`
### lidar
`roslaunch team_one lidar.launch`

## Mapping
### Cartographer
`roslaunch team_one cartographer.launch`

### Save map
`rosrun map_server map_server <map_path>`

`roslaunch scout_mini_2dnav move_base_only.launch`

## Localization
### AMCL
`roslaunch scout_mini_2dnav amcl.launch`

`rosservice call /global_localization`

# Simulator
## TF, odom, robot state publisher
`roslaunch scout_sim bringup_simulator_robot.launch`

