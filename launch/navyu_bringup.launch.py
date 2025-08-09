# Copyright 2024 RyuYamamoto.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    rviz_config = PathJoinSubstitution([FindPackageShare("kachaka_navyu"), "rviz", "kachaka_navyu.rviz"])

    navyu_config_path = PathJoinSubstitution(
        [FindPackageShare("kachaka_navyu"), "config", "navyu_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),

            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="navyu_global_costmap_node",
                remappings=[("/map", "/kachaka/mapping/map"), ("/costmap", "/global_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="navyu_local_costmap_node",
                remappings=[("/costmap", "/local_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_planner",
                executable="navyu_global_planner_node",
                name="navyu_global_planner_node",
                remappings=[("/goal_pose", "/goal_pose_1"), ("/costmap", "/global_costmap")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_tracker",
                executable="navyu_path_tracker_node",
                name="navyu_path_tracker_node",
                remappings=[("/cmd_vel", "cmd_vel_in")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_safety_limiter",
                executable="navyu_safety_limiter_node",
                name="navyu_safety_limiter_node",
                remappings=[("/cmd_vel_out", "/kachaka/manual_control/cmd_vel")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
