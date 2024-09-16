from collections import deque 
import heapq
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
def get_astar_search_path(adj_matrix, node_attributes, start_node, goal_node):
    def heuristic(n1, n2):
        """ Calculate the Euclidean distance between two nodes based on (x, y) coordinates. """
        x1, y1 = float(node_attributes[n1]['x']), float(node_attributes[n1]['y'])
        x2, y2 = float(node_attributes[n2]['x']), float(node_attributes[n2]['y'])
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    frontier = [(0, start_node)]  # Priority queue of (estimated total cost, current node)
    came_from = {start_node: None}  # Tracks the path back to the start.
    cost_so_far = {start_node: 0}   # Tracks the cost of reaching each node.

    while frontier:
        _, current = heapq.heappop(frontier)  # Get the node with the lowest total estimated cost

        # If we reached the goal, reconstruct the path
        if current == goal_node:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # We reverse the path to get start → goal

        # Explore neighbors of 'current' node
        for neighbor, connected in enumerate(adj_matrix[current]):
            if connected:  # If there's an edge between current and neighbor
                new_cost = cost_so_far[current] + connected  # Actual cost to reach 'neighbor'
                
                # If the neighbor hasn't been visited or we've found a cheaper path to it
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal_node)  # A* scoring: g(n) + h(n)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

    return None 

start_node=1
goal_node= 3
get_astar_search_path(adj_matrix, start_node, goal_node)