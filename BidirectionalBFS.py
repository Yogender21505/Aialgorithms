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
def get_bidirectional_search_path(adj_matrix, start_node, goal_node):
    if start_node == goal_node:
        return [start_node]

    # Helper function to perform one step of BFS from either direction
    def bfs_step(frontier, visited, parent, other_visited):
        current_node = frontier.popleft()
        # Explore the neighbors of the current node
        for neighbor, connected in enumerate(adj_matrix[current_node]):
            if connected > 0 and neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current_node
                frontier.append(neighbor)
                # If this node was visited by the other search
                if neighbor in other_visited:
                    return neighbor
        return None

    # Initialize frontiers, visited sets, and parent maps for both searches
    forward_frontier = deque([start_node])
    backward_frontier = deque([goal_node])
    forward_visited = {start_node}
    backward_visited = {goal_node}
    forward_parent = {start_node: None}
    backward_parent = {goal_node: None}

    meet_node = None

    # Perform BFS from both sides
    while forward_frontier and backward_frontier:
        # Step forward BFS
        meet_node = bfs_step(forward_frontier, forward_visited, forward_parent, backward_visited)
        if meet_node:
            break

        # Step backward BFS
        meet_node = bfs_step(backward_frontier, backward_visited, backward_parent, forward_visited)
        if meet_node:
            break

    # If no meeting point was found, return None
    if meet_node is None:
        return None

    # Reconstruct the path from start to goal via the meeting node
    return reconstruct_bidirectional_path(forward_parent, backward_parent, meet_node)

def reconstruct_bidirectional_path(forward_parent, backward_parent, meet_node):
    # Rebuild the path from start_node to meet_node using forward_parent
    path = []
    node = meet_node
    while node is not None:
        path.append(node)
        node = forward_parent[node]

    # Reverse the forward path to get it in correct order
    path = path[::-1]

    # Rebuild the path from meet_node to goal_node using backward_parent
    node = backward_parent[meet_node]
    while node is not None:
        path.append(node)
        node = backward_parent[node]

    return path

start_node=1
goal_node= 3
get_bidirectional_search_path(adj_matrix, start_node, goal_node)