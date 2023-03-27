#!/usr/bin/env python3

import rospy
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Header
from nav_msgs.msg import OccupancyGrid

def visualize_maze(data):
    # convert the maze message to a numpy array
    maze_dimensions = rospy.get_param('maze_dimensions')
    maze_dimensions = json.loads(maze_dimensions)
    num_rows = int(maze_dimensions[0])
    num_cols = int(maze_dimensions[1])
    maze = np.array(data.data, dtype=np.float32).reshape(num_rows, num_cols)

    # create an OccupancyGrid message with the same dimensions as the maze
    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1.0
    grid.info.width = maze.shape[1]
    grid.info.height = maze.shape[0]
    grid.info.origin.position.x = -(maze.shape[1] / 2.0)
    grid.info.origin.position.y = -(maze.shape[0] / 2.0)

    # convert the maze to an occupancy grid
    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            if maze[i][j] == 1:
                index = j + i * maze.shape[1]
                grid.data.append(100)
            else:
                grid.data.append(0)

    # publish the occupancy grid message
    print('Publish as occupancy grid ...')
    pub.publish(grid)

if __name__ == '__main__':
    # initialize the ROS node
    rospy.init_node('maze_visualizer')

    # create a publisher for the occupancy grid
    pub = rospy.Publisher('maze_grid', OccupancyGrid, queue_size=10)

    # create a subscriber for the maze
    sub = rospy.Subscriber('maze_matrix', Float32MultiArray, visualize_maze)

    # spin the node
    rospy.spin()
