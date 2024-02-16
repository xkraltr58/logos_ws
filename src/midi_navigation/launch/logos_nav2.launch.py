import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()

    
    amcl_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/amcl_params.yaml'
    bt_navigator_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/behavior_tree_navigator_params.yaml'
    collision_monitor_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/collision_monitor_params.yaml'
    costmap_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/costmap_params.yaml'
    planner_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/planner_params.yaml'
    dwb_controller_params_file = '/home/furkan/logos_ws/src/midi_navigation/config/dwb_controller_params.yaml'
    

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file]
    )
    ld.add_action(amcl_node)

    # BT navigator node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_params_file]
    )
    ld.add_action(bt_navigator_node)

    # Collision monitor node
    collision_monitor_node = Node(
        package='nav2_controller',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_monitor_params_file]
    )
    ld.add_action(collision_monitor_node)

    # Global costmap node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='costmap',
        output='screen',
        parameters=[costmap_params_file]  
    )
    ld.add_action(costmap_node)

    # Planner server node
    planner_server_node = Node(
        package='nav2_navfn_planner',
        executable='navfn_planner',
        name='planner_server',
        output='screen',
        parameters=[planner_params_file] 
    )
    ld.add_action(planner_server_node)

    # DWB planner node
    dwb_planner_node = Node(
        package='nav2_dwb_controller',
        executable='dwb_local_planner',
        name='dwb_local_planner',
        output='screen',
        parameters=[dwb_controller_params_file] 
    )
    ld.add_action(dwb_planner_node)


    
    return ld
