from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot
import time

delay = 0.0001

def reset(visited: set[Spot]) -> None:
    for spot in visited:
        if not spot.is_start() and not spot.is_end():
            spot.reset()

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    queue = deque()
    queue.append(start)
    visited = set()
    visited.add(start)
    parent = {}
    while queue:
        current = queue.popleft()
        if current == end:
            # reconstruct path
            while current != start:
                current = parent[current]
                if current is not start and current is not end:
                    current.make_path()
                draw()
                time.sleep(delay)
            reset(visited)
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                queue.append(neighbor)
                if neighbor is not start and neighbor is not end:
                    neighbor.make_open()
                draw()
                time.sleep(delay)
                parent[neighbor] = current
    reset(visited)
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    stack = []
    stack.append(start)
    visited = set()
    visited.add(start) 
    parent = {}
    while stack:
        current = stack.pop()
        if current == end:
            # reconstruct path
            while current != start:
                current = parent[current]
                if current is not start and current is not end:
                    current.make_path()
                draw()
                time.sleep(delay)
            reset(visited)
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                stack.append(neighbor)
                if neighbor is not start and neighbor is not end:
                    neighbor.make_open()
                draw()
                time.sleep(delay)
                parent[neighbor] = current
    reset(visited)
    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start)) 
    came_from = {}
    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid.grid for spot in row}
    f_score[start] = h_manhattan_distance((start.row, start.col), (end.row, end.col))
    visited = {start}
    while not open_set.empty():
        current = open_set.get()[2]
        visited.remove(current)
        if current == end:
            while current in came_from:
                current = came_from[current]
                if current is not start and current is not end:
                    current.make_path()
                draw()
                time.sleep(delay)
            return True
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h_manhattan_distance((neighbor.row, neighbor.col), (end.row, end.col))
                if neighbor not in visited:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    visited.add(neighbor)
                    if neighbor is not start and neighbor is not end:
                        neighbor.make_open()
                draw()
                time.sleep(delay)
    pass

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss