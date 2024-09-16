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
def get_bidirectional_heuristic_search_path(adj_matrix, node_attributes, start_node, goal_node):
    def heuristic(n1, n2):
        """Calculate the Euclidean heuristic between two nodes (n1 and n2) based on coordinates."""
        x1, y1 = float(node_attributes[n1]['x']), float(node_attributes[n1]['y'])
        x2, y2 = float(node_attributes[n2]['x']), float(node_attributes[n2]['y'])
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Priority queues for forward and backward searches
    forward_frontier = [(0, start_node)]
    backward_frontier = [(0, goal_node)]

    # Path dictionaries and cost trackers for both directions
    forward_prev = {start_node: None}
    backward_prev = {goal_node: None}
    forward_cost = {start_node: 0}
    backward_cost = {goal_node: 0}

    meet_node = None  # Node where the two searches meet

    # Continue search as long as both frontiers aren't empty
    while forward_frontier and backward_frontier:
        # Forward search step
        if forward_frontier:
            current_f_cost, current_f_node = heapq.heappop(forward_frontier)
            for neighbor, connected in enumerate(adj_matrix[current_f_node]):
                if connected > 0:
                    new_cost = forward_cost[current_f_node] + connected  # Cost to reach neighbor
                    if neighbor not in forward_cost or new_cost < forward_cost[neighbor]:
                        forward_cost[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, goal_node)  # g(n) + h(n)
                        heapq.heappush(forward_frontier, (priority, neighbor))
                        forward_prev[neighbor] = current_f_node
                        if neighbor in backward_prev:  # Check if there is a meeting point
                            meet_node = neighbor
                            return reconstruct_bidirectional_path(forward_prev, backward_prev, meet_node)

        # Backward search step
        if backward_frontier:
            current_b_cost, current_b_node = heapq.heappop(backward_frontier)
            for neighbor, connected in enumerate(adj_matrix[current_b_node]):
                if connected > 0:
                    new_cost = backward_cost[current_b_node] + connected  # Cost to reach neighbor
                    if neighbor not in backward_cost or new_cost < backward_cost[neighbor]:
                        backward_cost[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, start_node)  # g(n) + h(n)
                        heapq.heappush(backward_frontier, (priority, neighbor))
                        backward_prev[neighbor] = current_b_node
                        if neighbor in forward_prev:  # Check if there is a meeting point
                            meet_node = neighbor
                            return reconstruct_bidirectional_path(forward_prev, backward_prev, meet_node)

    # If no path is found
    return None

def reconstruct_bidirectional_path(forward_prev, backward_prev, meet_node):
    """Reconstructs the final path when both forward and backward searches meet at meet_node."""
    # Traverse backwards from the meet_node to the start_node using forward_prev
    path = []
    node = meet_node
    while node is not None:
        path.append(node)
        node = forward_prev[node]
    path = path[::-1]  # Reverse the forward path to get correct order from start

    # Add path from meet_node to goal_node by traversing backward_prev
    node = backward_prev[meet_node]
    while node is not None:
        path.append(node)
        node = backward_prev[node]

    return path

start_node=1
goal_node= 3
get_bidirectional_heuristic_search_path(adj_matrix, start_node, goal_node)