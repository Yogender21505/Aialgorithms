from collections import deque 

import numpy as np
import matplotlib.pyplot as plt
from collections import deque

adj_matrix = np.array([
    [0, 1, 0, 0, 1],  # Node 0 connects to 1 and 4
    [1, 0, 1, 0, 0],  # Node 1 connects to 0 and 2
    [0, 1, 0, 1, 1],  # Node 2 connects to 1, 3, and 4
    [0, 0, 1, 0, 1],  # Node 3 connects to 2 and 4
    [1, 0, 1, 1, 0]   # Node 4 connects to 0, 2, and 3
])
node_attributes = {
    0: {'x': '1.0', 'y': '1.0'},
    1: {'x': '2.0', 'y': '2.0'},
    2: {'x': '3.0', 'y': '3.0'},
    3: {'x': '4.0', 'y': '4.0'},
    4: {'x': '5.0', 'y': '5.0'}
}
def depth_limited_search(adj_matrix, start_node, goal_node, depth, visited):

    if start_node == goal_node:
        return [start_node]
    
    if depth == 0:  # Reached depth limit
        return None

    visited.add(start_node)  # Mark the node as visited

    # Explore neighbors
    for neighbor, connected in enumerate(adj_matrix[start_node]):
        if connected and neighbor not in visited:  # If connected and not visited
            result = depth_limited_search(adj_matrix, neighbor, goal_node, depth - 1, visited)
            if result:  # If a valid path is found, return the path
                return [start_node] + result

    visited.remove(start_node)  # Unmark this node during backtracking
    return None


def check_existence(adj_matrix, start, goal):

    visited = set()  # Track visited nodes to avoid cycles
    stack = [start]

    while stack:
        node = stack.pop()
        if node == goal:
            return True
        
        visited.add(node)

        # Pushing adjacent nodes (neighbors) into the stack
        for neighbor, connected in enumerate(adj_matrix[node]):
            if connected and neighbor not in visited:
                stack.append(neighbor)

    return False


def get_ids_path(adj_matrix, start_node, goal_node):

    max_depth = len(adj_matrix)
    
    # First, check if there is any possible path between the start and goal.
    if check_existence(adj_matrix, start_node, goal_node):
        # If a path exists, run Iterative Deepening Search
        for depth in range(max_depth):
            visited = set()  # Reset visited nodes for each depth limit
            result = depth_limited_search(adj_matrix, start_node, goal_node, depth, visited)
            if result:
                return result
    
    return None 

start_node=1
goal_node= 3
depth_limited_search(adj_matrix, start_node, goal_node)