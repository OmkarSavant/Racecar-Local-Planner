# Racecar-Local-Planner-
ROS package for a motion planner used in head-to-head autonomous racing.

Link to our paper describing this project: https://www.dropbox.com/s/mad0hyl8c2wwbm5/Robo%20Final%20Paper.pdf?dl=0

Working video of this project: https://www.youtube.com/watch?v=xG9_DVODh0s

This is an online planner for use in a head- to-head autonomous race. Traditional approaches to autonomous racing involve using methods such as Pure Pursuit or Model Predictive Control to follow set waypoints or racing lines around a course. However, when another car is introduced in the field, there are many cases where a certain waypoint may be covered or the path to the waypoint may be inaccessible. This is a local planner that operates above 900 Hz and uses LiDAR data to sense obstacles and plan around them to reach waypoints, ultimately publishing a trajectory for the car to follow. This planner has been built into a ROS package and is written using C++.




