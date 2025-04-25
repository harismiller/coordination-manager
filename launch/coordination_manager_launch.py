from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

import os

def generate_agent_nodes(context):
    num_agents = int(LaunchConfiguration('num_agents').perform(context))
    db_directory = LaunchConfiguration('db_directory').perform(context)
    nodes = []
    for i in range(num_agents):
        agent_node = GroupAction([
            PushRosNamespace(f'robot{i+1}'),
            Node(
                package='coordination-manager',
                executable='planner_parser_node',
                output='screen',
                parameters=[{'db_directory': db_directory}]
            )
        ])
        nodes.append(agent_node)
    return nodes

def generate_launch_description():

    current_file_dir = os.path.dirname(os.path.realpath(__file__))
    db_dir = os.path.join(current_file_dir.split('/install')[0], 'src/coordination-manager/db')
    # Declare the number of agents
    declare_num_agents = DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Number of agents to launch'
    )

    # Declare the database directory
    declare_db_directory = DeclareLaunchArgument(
        'db_directory',
        default_value=db_dir,
        description='Path to the database directory'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(declare_num_agents)
    ld.add_action(declare_db_directory)

    # Add the system_compiler_node
    system_compiler_node = Node(
        package='coordination-manager',
        executable='system_compiler_node',
        name='system_compiler_node',
        output='screen'
    )
    ld.add_action(system_compiler_node)

    # Add the agent nodes
    ld.add_action(OpaqueFunction(function=generate_agent_nodes))

    return ld