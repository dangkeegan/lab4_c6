import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Find the location of the package on your computer
    rosbot_pkg_path = get_package_share_directory('rosbot')
    rosbot_rosbot_nav_pkg_path = get_package_share_directory('rosbot_nav')
    rosbot_nav_pkg_path = get_package_share_directory('nav2_bringup')
    rosbot_arm_pkg_path = get_package_share_directory('rosbot_arm')



    # Launch another launch file.
    # This is the same as running
    # `ros2 launch example_pkg example_launch.py`
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_pkg_path, 'launch', 'sensors_launch.py')),
    )
    nav_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_rosbot_nav_pkg_path, 'launch', 'rviz_launch.py')),
    )
    nav_launch = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_nav_pkg_path, 'launch', 'bringup_launch.py')),
    )
    arm_launch = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource(
            os.path.join(rosbot_arm_pkg_path, 'launch', 'arm_launch.py')),
    )
    # Your final returned LaunchDescription includes 
    # a list of all of the nodes, launch files, etc.
    return launch.LaunchDescription([
        sensors_launch,
        nav_launch,
        nav_rviz_launch,
        arm_launch
    ])


if __name__ == '__main__':
    # Create a LaunchDescription object
    ld = generate_launch_description()

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()