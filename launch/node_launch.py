from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="beginner_tutorials",
            executable="node_pubs",
            name="node_pubs",
        ),

        Node(
            package="beginner_tutorials",
            executable="node_subs",
            name="node_subs",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "Mountain"}, #command line parameter declaration with the data as "Mountain"
            ]   
        )
    ])