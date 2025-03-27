Import TurtleBot3 packages with vcs:
vcs import . < turtlebot3.repos

ros2 launch tb3_sim turtlebot3_world.launch.py

ros2 launch tb3_sim nav2.launch.py

ros2 launch tb3_autonomy autonomy.launch.py