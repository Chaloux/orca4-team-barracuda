#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Run the "huge loop" mission

Code inspired by https://github.com/ros2/ros2cli/blob/rolling/ros2action/ros2action/verb/send_goal.py

Usage:
-- ros2 run orca_bringup mission_runner.py
"""

from enum import Enum

import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header, String

import math

class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception)


def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))

def generate_lawnmower_waypoints(start_x, start_y, width, height, num_rows, depth):
    waypoints = []
    row_spacing = height / (num_rows - 1)
    
    for i in range(num_rows):
        y = start_y + i * row_spacing
        if i % 2 == 0:
            waypoints.append(make_pose(start_x, y, depth))
            waypoints.append(make_pose(start_x + width, y, depth))
        else:
            waypoints.append(make_pose(start_x + width, y, depth))
            waypoints.append(make_pose(start_x, y, depth))
    
    return waypoints

# Go to AUV mode
go_auv = TargetMode.Goal()
go_auv.target_mode = TargetMode.Goal.ORCA_MODE_AUV

# Go to ROV mode
go_rov = TargetMode.Goal()
go_rov.target_mode = TargetMode.Goal.ORCA_MODE_ROV

# Go home (1m deep)
go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))


<<<<<<< HEAD
    


def make_grid(goal, x1: float, y1: float, x2: float, y2: float, grid_z: float, spacing: float, num_transects: int):
    """
    Makes a grid survey pattern of parallel transects
    :param FollowWaypoints.Goal: the goal to which this func will append poses
    :param float x1: x component of start point of first transect
    :param float y1: y component of start point of first transect
    :param float x2: x component of end point of first transect
    :param float y2: y component of end point of first transect
    :param float z: the depth of the grid survey
    :param float spacing: distance between transects, positive is to right of first transect
    :param int num_transects how many to generate 
    """
    transect_delta_x = x2 - x1
    transect_delta_y = y2 - y1

    theta = math.atan2(transect_delta_x, transect_delta_y)

    transit_delta_x = spacing * math.cos(theta)
    transit_delta_y = -spacing * math.sin(theta)


    # lay down the first transect
    goal.poses.append(make_pose(x=x1, y=y1, z=grid_z))
    goal.poses.append(make_pose(x=x2, y=y2, z=grid_z))

    current_x = x2
    current_y = y2

    transect_direction_multiplier = 1.0

    # lay down the remainder of the transects
    for _ in range(num_transects):
        current_x += transit_delta_x
        current_y += transit_delta_y
        goal.poses.append(make_pose(x=current_x, y=current_y, z=grid_z))

        transect_direction_multiplier *= -1.0

        current_x += transect_direction_multiplier * transect_delta_x
        current_y += transect_direction_multiplier * transect_delta_y
        goal.poses.append(make_pose(x=current_x, y=current_y, z=grid_z))


grid_pattern = FollowWaypoints.Goal()
make_grid(grid_pattern, x1=0.0, y1=1.0, x2=5.0, y2=-2.0, grid_z=-7.0, spacing=2.0, num_transects=3)



=======
# Lawnmower pattern
lawnmower_waypoints = generate_lawnmower_waypoints(0.0, 0.0, 20.0, 20.0, 5, -7.0)
lawnmower_mission = FollowWaypoints.Goal()
lawnmower_mission.poses.extend(lawnmower_waypoints)
>>>>>>> ef774aac56ee4d82b9c63d38a000da26f909545c

# Send a goal to an action server and wait for the result.
# Cancel the goal if the user hits ^C (KeyboardInterrupt).
def send_goal(node, action_client, send_goal_msg) -> SendGoalResult:
    goal_handle = None

    try:
        action_client.wait_for_server()

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg)
        rclpy.spin_until_future_complete(node, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError('Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

        print('Goal completed')
        return SendGoalResult.SUCCESS

    except KeyboardInterrupt:
        # Cancel the goal if it's still active
        # TODO(clyde): this seems to work, but a second exception is generated -- why?
        if (goal_handle is not None and
                (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                 GoalStatus.STATUS_EXECUTING == goal_handle.status)):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                raise RuntimeError('Exception while canceling goal: {!r}'.format(cancel_future.exception()))

            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            if len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            if cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')

            print('Goal canceled')
            return SendGoalResult.CANCELED
        
def qr_found_callback(some_node, qr_found_msg):
    print('Got a qrFound message: "%s"' % qr_found_msg.data)
    if (qr_found_msg.data == "found it!"):
        send_goal(node, set_target_mode, go_rov)

<<<<<<< HEAD
node = None
set_target_mode = None

=======
>>>>>>> ef774aac56ee4d82b9c63d38a000da26f909545c
def main():
    follow_waypoints = None

    rclpy.init()

    try:
        node = rclpy.create_node("mission_runner")

        set_target_mode = ActionClient(node, TargetMode, '/set_target_mode')
        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')
        #node.create_subscription(String, "qrFound", qr_found_callback, 10)

        print('>>> Setting mode to AUV <<<')
        if send_goal(node, set_target_mode, go_auv) == SendGoalResult.SUCCESS:
<<<<<<< HEAD
            print("Grid mission waypoints:")
            for poseStamped in grid_pattern.poses:
                print("    {: .2f}, {: .2f}, {: .2f}".format(poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z))
            print('>>> Executing grid mission <<<')
            #send_goal(node, follow_waypoints, delay_loop)
            send_goal(node, follow_waypoints, grid_pattern)
=======
            print('>>> Executing lawnmower mission <<<')
            send_goal(node, follow_waypoints, lawnmower_mission)
>>>>>>> ef774aac56ee4d82b9c63d38a000da26f909545c

            print('>>> Setting mode to ROV <<<')
            send_goal(node, set_target_mode, go_rov)

            print('>>> Mission complete <<<')
        else:
            print('>>> Failed to set mode to AUV, quit <<<')

    finally:
        if set_target_mode is not None:
            set_target_mode.destroy()
        if follow_waypoints is not None:
            follow_waypoints.destroy()
        if node is not None:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
