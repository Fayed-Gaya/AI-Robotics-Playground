#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import time

def occupancy_grid_to_gazebo_world(msg):
    # Set up Gazebo service proxies
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Get current time for header
    now = rospy.Time.now()

    # Set up header
    header = Header()
    header.stamp = now
    header.frame_id = 'world'

    # Set up pose for unit box
    pose = Pose()
    pose.orientation.w = 1.0

    # Set up box dimensions
    box_width = msg.info.resolution
    box_height = msg.info.resolution
    box_length = msg.info.resolution

    # Loop through occupancy grid and create unit boxes for occupied cells
    for i in range(msg.info.height):
        for j in range(msg.info.width):
            # Get occupancy value of cell
            index = j + i * msg.info.width
            occupancy = msg.data[index]

            # Create box for occupied cell
            if occupancy >= 50:
                # Set up pose for box
                pose.position.x = msg.info.origin.position.x + j * box_width
                pose.position.y = msg.info.origin.position.y + i * box_height
                pose.position.z = box_height / 2.0

                # Set up SDF model XML
                model_xml = '<sdf version="1.5"><model name="box"><link name="link"><collision name="collision"><geometry><box><size>{:.3f} {:.3f} {:.3f}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{:.3f} {:.3f} {:.3f}</size></box></geometry></visual></link></model></sdf>'.format(box_width, box_length, box_height, box_width, box_length, box_height)

                # Spawn box in Gazebo
                spawn_model('box_{}_{}'.format(i, j), model_xml, '', pose, header)

    # Wait for boxes to spawn before continuing
    time.sleep(1)

    # Delete all boxes after a certain time
    delete_time = now + rospy.Duration.from_sec(30.0)
    while rospy.Time.now() < delete_time:
        # Get current state of models
        models = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        # Delete all box models
        for name in models.name:
            if name.startswith('box_'):
                delete_model(name)

        # Wait a little before checking again
        time.sleep(1)

if __name__ == '__main__':

    # Set up ROS node
    rospy.init_node('occupancy_grid_to_gazebo_world')

    # Subscribe to occupancy grid topic
    rospy.Subscriber('maze_grid', OccupancyGrid, occupancy_grid_to_gazebo_world)

    # Spin and wait
    rospy.spin()
