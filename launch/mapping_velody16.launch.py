from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="False", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "velody16.yaml"]),
        description="Path to the Point-LIO config file",
    )

    laser_mapping_params = [
        point_lio_cfg_dir,
        {
            "use_imu_as_input": False,  # Change to True to use IMU as input of Point-LIO
            "prop_at_freq_of_imu": True,
            "check_satu": True,
            "init_map_size": 10,
            "point_filter_num": 4,  # Options: 4, 3
            "space_down_sample": True,
            "filter_size_surf": 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            "filter_size_map": 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            "ivox_nearby_type": 6,  # Options: 0.5, 0.3, 0.15, 0.1
            "runtime_pos_log_enable": False,  # Option: True
        },
    ]

    start_velodyne_convert_tool = Node(
        package="isaac_sim_pointcloud_tool",
        executable="converter",
        namespace=namespace,
        remappings=remappings,
        output="screen",
    )

    start_laser_mapping_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=laser_mapping_params,
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_laser_mapping_node)
    ld.add_action(start_rviz_node)

    return ld
