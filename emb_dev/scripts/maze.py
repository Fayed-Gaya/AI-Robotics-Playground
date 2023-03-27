#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt

def visualize_maze(maze):
    # create a figure and axis object
    fig, ax = plt.subplots()

    # create a color map for the walls and the paths
    cmap = plt.get_cmap('binary', 2)

    # plot the maze as a image
    ax.imshow(maze, cmap=cmap, interpolation='nearest')

    # set the ticks to show only at integer values and label them
    ax.set_xticks(np.arange(-0.5, maze.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, maze.shape[0], 1), minor=True)
    ax.set_xticklabels(np.arange(0, maze.shape[1]+1, 1))
    ax.set_yticklabels(np.arange(0, maze.shape[0]+1, 1))
    # set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    # set the grid lines to be white and only show for integer values
    ax.grid(which='minor', color='w', linestyle='-', linewidth=1)

    # set the axis limits to show the whole maze
    ax.set_xlim([-0.5, maze.shape[1]-0.5])
    ax.set_ylim([-0.5, maze.shape[0]-0.5])

    # set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    # ax.axis('off')

    # show the plot
    plt.show()

def generate_maze(length, width, max_room_size=5, room_density=0.1):
    # initialize the maze with all walls
    maze = np.ones((length, width), dtype=int)

    # create the outer perimeter of wall cells
    maze[0,:] = 1; maze[-1,:] = 1; maze[:,0] = 1; maze[:,-1] = 1

    # set the top-left cell as the starting point
    maze[1,1] = 0

    # perform a randomized depth-first search to create paths in the maze
    stack = [(1, 1)]
    while stack:
        current_row, current_col = stack.pop()
        neighbors = get_unvisited_neighbors(current_row, current_col, maze)

        if neighbors:
            # choose a random neighbor to visit
            neighbor_row, neighbor_col = neighbors[np.random.randint(len(neighbors))]
            # carve a path to the neighbor
            maze[current_row + (neighbor_row - current_row)//2, current_col + (neighbor_col - current_col)//2] = 0
            maze[neighbor_row, neighbor_col] = 0
            stack.append((current_row, current_col))
            stack.append((neighbor_row, neighbor_col))

    # randomly place rooms in the maze
    for i in range(2, length-2):
        for j in range(2, width-2):
            if maze[i, j] == 0 and np.random.rand() < room_density:
                room_length = np.random.randint(1, max_room_size+1)
                room_width = np.random.randint(1, max_room_size+1)
                if i + room_length < length and j + room_width < width:
                    maze[i:i+room_length, j:j+room_width] = 0

    return maze

def get_unvisited_neighbors(row, col, maze):
    neighbors = []
    directions = [(0, -2), (0, 2), (-2, 0), (2, 0)]
    for direction in directions:
        neighbor_row = row + direction[0]
        neighbor_col = col + direction[1]
        if (0 <= neighbor_row < maze.shape[0] and
            0 <= neighbor_col < maze.shape[1] and
            maze[neighbor_row, neighbor_col] == 1):
            neighbors.append((neighbor_row, neighbor_col))
    return neighbors


def publish_maze(maze):
    # initialize the ROS node
    rospy.init_node('maze')

    # create a publisher for the "maze" topic
    pub = rospy.Publisher('maze_matrix', Float32MultiArray, queue_size=10)

    # set the publishing rate to 1 Hz
    rate = rospy.Rate(1)

    # publish the maze every second
    while not rospy.is_shutdown():
        maze_msg = Float32MultiArray()
        maze_msg.data = maze.flatten().tolist()
        pub.publish(maze_msg)
        print(f'Published maze: {maze.shape[0]} x {maze.shape[1]}')
        rate.sleep()


if __name__ == '__main__':
    try:
        # generate the maze
        maze_dimensions = rospy.get_param('maze_dimensions')
        maze_dimensions = json.loads(maze_dimensions)
        num_rows = int(maze_dimensions[0])
        num_cols = int(maze_dimensions[1])
        rospy.logdebug('Creating Maze: %dx%d', num_rows, num_cols)
        maze = generate_maze(num_rows, num_cols, max_room_size=3, room_density=0.1)


        # DEBUG: visulize the maze
        # visualize_maze(maze)
        
        # publish the maze
        publish_maze(maze)
    except rospy.ROSInterruptException:
        pass


