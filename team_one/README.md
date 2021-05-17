# AMCL
## Tuning guide
odom_model_type: diff -> diff-corrected improved a lot  

transform_tolerance: decreasing this values, makes a lot better. but increases the computation
- if extrapolation occurs increases this value

resample_interval: increasing this values, allows divergence happens faster

recovery_alpha_slow: changing this value did not do much, so i have changed to default

odom_alpha1: increase this value if rotation noise is expected from rotation

odom_alpha4: increase this value if translation noise is expected from rotation


# move_base
## Tuning guide
# recovery behavior
- consider making custom recovery behavior

# Costmap
## common
### general
robot_radius: radius required for robot to turn
### inflation layer
The advantage is that the robot would prefer to move in the middle of obstacles.

Gray: safe area(free space), robot movement will not collide in this area

Blue: the risk area where robots may clooide

Cyon: dangerous area, where if the center of robot meets cyon, it collides

Purple: wall


### Tips
so basically, if the laser reading is equal to max_range, it takes no effect
Thus, we need to set max_range reading to inf, and allow obstacle_layer to clear inf by setting inf_is_valid to true
So, we can change the max_range reading to inf using laserfilters

# planner
## DWA
### collision avoidance
- occdist_scale: increase this value to avoid obstacle
- path_distance_bias: increase this value to follow the global_plan


### goal reached
- yaw_goal_tolerance: increase this value to lessen the rotation upon arrival