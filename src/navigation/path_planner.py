import heapq
import numpy as np

def a_star(start, goal, grid):
    """A* algorithm for path planning."""
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            break

        for neighbor in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_list, (priority, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal)

# Utility functions: heuristic, get_neighbors, reconstruct_path...
