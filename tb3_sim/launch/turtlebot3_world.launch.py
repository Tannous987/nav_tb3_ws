import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  launch_file_dir = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'), 'launch')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  pkg_tb3_sim = get_package_share_directory('tb3_sim')

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  x_pose = LaunchConfiguration('x_pose', default='-2.0')
  y_pose = LaunchConfiguration('y_pose', default='-0.5')

  world = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'worlds',
      'turtlebot3_world.world'
  )

  gzserver_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
      ),
    launch_arguments={
        'world': world,
        'verbose': 'false',
        'physics': 'ode',
        'paused': 'false',
        'use_sim_time': 'true',
        'extra_gazebo_args': '--lockstep --update-rate 100'  # clearly reduce from default (~1000) to ~100
    }.items()
  )

  gzclient_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
      )
  )

  robot_state_publisher_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
      ),
      launch_arguments={'use_sim_time': use_sim_time}.items()
  )

  spawn_turtlebot_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
      ),
      launch_arguments={
          'x_pose': x_pose,
          'y_pose': y_pose
      }.items()
  )

  ld = LaunchDescription()

  ld.add_action(gzserver_cmd)
  ld.add_action(gzclient_cmd)
  ld.add_action(robot_state_publisher_cmd)
  ld.add_action(spawn_turtlebot_cmd)

  return ld
