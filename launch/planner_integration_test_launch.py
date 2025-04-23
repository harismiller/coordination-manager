from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_agent_nodes(context):
    num_agents = int(LaunchConfiguration('num_agents').perform(context))
    nodes = []
    for i in range(num_agents):
        agent_node = GroupAction([
            PushRosNamespace(f'robot{i+1}'),
            Node(
                package='coordination-manager',
                executable='status_agent_fail_publisher',
                output='screen'
            )
        ])
        nodes.append(agent_node)
    return nodes

def generate_launch_description():
    declare_num_agents = DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Number of agents to launch'
    )

    ld = LaunchDescription()

    ld.add_action(declare_num_agents)

    ld.add_action(OpaqueFunction(function=generate_agent_nodes))

    return ld