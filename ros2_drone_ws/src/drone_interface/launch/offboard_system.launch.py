from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path for MAVROS TF configuration
    drone_if_share = get_package_share_directory('drone_interface')
    mavros_tf_file = os.path.join(drone_if_share, 'config', 'mavros_tf.yaml')

    # ---------------- MAVROS NODE ----------------
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            mavros_tf_file,
            {'fcu_url': 'udp://:14540@localhost:14557'}
        ]
    )

    # ---------------- TRAJECTORY GENERATOR ----------------
    traj = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            'type': 'helix',
            'hz': 30.0,
            'radius': 3.0,
            'height': 6.0,
            'speed': 0.6
        }]
    )

    # ---------------- AFONFTSM CONTROLLER ----------------
    controller = Node(
        package='afonftsm_controller',
        executable='afonftsm_controller_node',
        name='afonftsm_controller',
        output='screen',
        parameters=[{
            'rate': 30.0,
            'memory_len': 60,
            'gamma_d': 0.9,
            'gamma_i': 0.1,
            'c_d': 0.18,
            'c_i': 0.18,
            'k1': 2.6,
            'k2': 0.22,
            'p_corr': 0.8,
            'v_max': 2.0,
            'publish_to_mavros': True
        }]
    )

    # ---------------- OFFBOARD NODE ----------------
    offb = Node(
        package='drone_interface',
        executable='offboard_node',
        name='offboard_node',
        output='screen'
    )

    return LaunchDescription([
        mavros,
        traj,
        controller,
        offb
    ])
