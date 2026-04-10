from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
	controllers_file = LaunchConfiguration("controllers_file")
	description_file = LaunchConfiguration("description_file")
	launch_rviz = LaunchConfiguration("launch_rviz")
	rviz_config_file = LaunchConfiguration("rviz_config_file")
	gazebo_gui = LaunchConfiguration("gazebo_gui")
	gazebo_verbosity = LaunchConfiguration("gazebo_verbosity")
	world_file = LaunchConfiguration("world_file")
	activate_joint_controller = LaunchConfiguration("activate_joint_controller")
	initial_joint_controller = LaunchConfiguration("initial_joint_controller")
	launch_payload_service = LaunchConfiguration("launch_payload_service")
	robot_spawn_x = LaunchConfiguration("robot_spawn_x")
	robot_spawn_y = LaunchConfiguration("robot_spawn_y")
	robot_spawn_z = LaunchConfiguration("robot_spawn_z")
	payload_tool_offset_x = LaunchConfiguration("payload_tool_offset_x")
	payload_tool_offset_y = LaunchConfiguration("payload_tool_offset_y")
	payload_tool_offset_z = LaunchConfiguration("payload_tool_offset_z")

	# Robot description from XACRO (pass controllers file into xacro)
	robot_description_content = Command(
		[
			PathJoinSubstitution([FindExecutable(name="xacro")]), " ", description_file, " ", "simulation_controllers:=", controllers_file,
		]
	)
	robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

	# Robot state publisher
	robot_state_publisher_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		output="both",
		parameters=[{"use_sim_time": True}, robot_description],
	)

	# RViz
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_config_file],
		parameters=[robot_description],
		condition=IfCondition(launch_rviz),
	)

	# Note: joint_state_broadcaster is automatically activated by controller_manager
	# Initial controller
	initial_joint_controller_spawner_started = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[initial_joint_controller, "-c", "/controller_manager"],
		condition=IfCondition(activate_joint_controller),
	)
	initial_joint_controller_spawner_stopped = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
		condition=UnlessCondition(activate_joint_controller),
	)


	gz_spawn_entity = Node(
		package="ros_gz_sim",
		executable="create",
		name="create",
		output="screen",
		arguments=[
			"-topic", "robot_description",
			"-name", "threelink_manipulator",
			"-x", robot_spawn_x,
			"-y", robot_spawn_y,
			"-z", robot_spawn_z,
		],
	)


	# Gazebo launch
	gz_launch_description = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
		launch_arguments={
			"gz_args": IfElseSubstitution(
				gazebo_gui,
				if_value=[" -r -v ", gazebo_verbosity, " ", world_file],
				else_value=[" -s -r -v ", gazebo_verbosity, " ", world_file],
			)
		}.items(),
	)

	# Gazebo <-> ROS bridge
	gz_sim_bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		arguments=[
			"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
			"/world/empty/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean",
			"/world/empty/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean",
			"/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity@gz.msgs.Entity@gz.msgs.Boolean",
			"/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean",
			# DetachableJoint expects ROS -> Gazebo for attach/detach triggers.
			"/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty",
			"/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty",
		],
		output="screen",
	)

	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
		output="screen",
	)

	static_world_base = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    output="log",
	)    

	payload_attach_server_node = Node(
		package="threelink_manipulator",
		executable="payload_attach_server.py",
		name="payload_attach_server",
		output="screen",
		parameters=[{
			"use_sim_time": True,
			"tool_frame": "tool0",
			"reference_frame": "world",
			"tool_offset_x": payload_tool_offset_x,
			"tool_offset_y": payload_tool_offset_y,
			"tool_offset_z": payload_tool_offset_z,
			"tool_offset_x_auto": True,
			"attach_clearance": 0.0,
			"forward_frame": "tool0",
			"use_forward_direction": True,
			"robot_spawn_offset_x": robot_spawn_x,
			"robot_spawn_offset_y": robot_spawn_y,
			"robot_spawn_offset_z": robot_spawn_z,
			"service_timeout_sec": 8.0,
			"pause_wait_sec": 1.0,
			"post_spawn_wait_sec": 0.5,
			"post_detach_wait_sec": 0.3,
			"detach_unpause_sec": 0.2,
			"attach_burst_count": 5,
			"detach_burst_count": 5,
			"burst_interval_sec": 0.05,
			"trigger_step_count": 5,
			"detach_before_attach": True,
			"marker_lifetime_sec": 0.5,
			"marker_delete_burst_count": 3,
			"marker_delete_interval_sec": 0.05,
		}],
		condition=IfCondition(launch_payload_service),
	)


	nodes_to_start = [
		robot_state_publisher_node,
		rviz_node,
		initial_joint_controller_spawner_stopped,
		initial_joint_controller_spawner_started,
		joint_state_broadcaster_spawner, 
		gz_spawn_entity,
		gz_launch_description,
		gz_sim_bridge,
		static_world_base,
		payload_attach_server_node,
	]

	return nodes_to_start


def generate_launch_description():
	declared_arguments = []
	declared_arguments.append(
		DeclareLaunchArgument(
			"controllers_file",
			default_value=PathJoinSubstitution([FindPackageShare("threelink_manipulator"), "config", "controllers.yaml"]),
			description="Absolute path to YAML file with the controllers configuration.",
		)
	)
	declared_arguments.append(
		DeclareLaunchArgument(
			"description_file",
			default_value=PathJoinSubstitution([FindPackageShare("threelink_manipulator"), "urdf", "threelink.urdf.xacro"]),
		)
	)
	declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz for visualization"))
	declared_arguments.append(
		DeclareLaunchArgument(
			"rviz_config_file",
			default_value=PathJoinSubstitution([FindPackageShare("threelink_manipulator"), "rviz", "view_robot.rviz"]),
		)
	)
	declared_arguments.append(DeclareLaunchArgument("gazebo_gui", default_value="false", description="Launch Gazebo with GUI"))
	declared_arguments.append(
		DeclareLaunchArgument(
			"gazebo_verbosity",
			default_value="1",
			description="Gazebo console verbosity level (0-4). Default 1 to reduce detachable-joint missing child warnings before payload spawn.",
		)
	)
	declared_arguments.append(DeclareLaunchArgument(
    "world_file",
    default_value=PathJoinSubstitution([
        FindPackageShare("threelink_manipulator"),
        "worlds",
        "freespace.sdf"
    ]),
    description="Gazebo world file",
))

	declared_arguments.append(DeclareLaunchArgument("activate_joint_controller", default_value="true", description="Activate initial controller"))
	declared_arguments.append(DeclareLaunchArgument("initial_joint_controller", default_value="forward_effort_controller", description="Initial controller to activate"))
	declared_arguments.append(DeclareLaunchArgument("launch_payload_service", default_value="true", description="Launch payload attach service node"))
	declared_arguments.append(DeclareLaunchArgument("robot_spawn_x", default_value="0.0", description="Spawn X position of the robot model"))
	declared_arguments.append(DeclareLaunchArgument("robot_spawn_y", default_value="0.0", description="Spawn Y position of the robot model"))
	declared_arguments.append(DeclareLaunchArgument("robot_spawn_z", default_value="3.0", description="Spawn Z position of the robot model"))
	declared_arguments.append(DeclareLaunchArgument("payload_tool_offset_x", default_value="0.0", description="Payload offset along tool frame X (ignored when tool_offset_x_auto:=true)"))
	declared_arguments.append(DeclareLaunchArgument("payload_tool_offset_y", default_value="0.0", description="Payload offset along tool frame Y"))
	declared_arguments.append(DeclareLaunchArgument("payload_tool_offset_z", default_value="0.0", description="Payload offset along tool frame Z"))

	return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
