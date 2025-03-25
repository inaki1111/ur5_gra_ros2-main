#!/usr/bin/env python3
import rclpy
import time
import pprint
from ur5e_moveit_client import UR5eMoveItClient

def main():
    print('initializing node...')
    rclpy.init()
    node = UR5eMoveItClient()
    pp = pprint.PrettyPrinter(indent=4)

    # Define the dictionary of poses
    poses = {
        'pose0': [-0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0],
        'pose1': [0.0, 0.7, 0.5, 1.0, 0.0, 0.0, 0.0],
        'pose2': [0.2, 0.7, 0.2, 1.0, 0.0, 0.0, 0.0]
    }

    # example 1: Retrieve and print the joint values and tool position
    print('Starting joint values and tool position')
    jp=node.get_joint_positions()
    pp.pprint(list(jp))

    link_name = 'tool0'
    position, orientation = node.get_link_position(link_name)
    if position and orientation:
        print(f'Position of {link_name}: {position}')
        print(f'Orientation of {link_name}: {orientation}')
    
    # example 2: Move the first joint in the direction of the workspace
    node.move_to_joints([1.57] + list(jp[1:]))

    jp=node.get_joint_positions()
    pp.pprint(list(jp))

    # example 3: Move the UR5e to the specified positions with a small pose
    # after each move command is completed
    for pose_name, pose in poses.items():
        node.move_to_pose(pose)
        print(f'{pose_name} done')
        time.sleep(1)

    # example 4: Move the UR5e to the specified positions asynchronously
    # and send the next move command before the previous one is completed
    # (only performs a small pose after each move command is issued)
    # This is useful when you want to move the robot to a series of poses

    for pose_name, pose in poses.items():
        node.move_to_pose_async(pose)
        print(f'{pose_name} done')
        time.sleep(0.1)

    print('Ending joint values and tool position')
    jp=node.get_joint_positions()
    pp.pprint(list(jp))
    link_name = 'tool0'
    position, orientation = node.get_link_position(link_name)
    if position and orientation:
        print(f'Position of {link_name}: {position}')
        print(f'Orientation of {link_name}: {orientation}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()