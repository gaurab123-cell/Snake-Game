import random
import time
import math
from collections import deque
from heapq import heappop, heappush

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Random movement algorithm (limited moves)
def random_move(start, goal, obstacles, rows, cols):
    path = []
    current = start

    for _ in range(1000):  # Limit to 1000 moves
        direction = random.choice(DIRECTIONS)
        new_pos = (current[0] + direction[0], current[1] + direction[1])

        if (
            0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
            new_pos not in obstacles
        ):
            path.append(direction)
            current = new_pos

        if current == goal:
            return path

    return []  # If it takes too long, return empty path


# Write your code below this only
# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    queue = deque([(start, [])])  # (current_position, path_taken)
    visited = set()
    visited.add(start)
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    
    while queue:
        (current, path) = queue.popleft()
        
        if current == goal:
            return path  # Return the shortest path to the food
        
        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])
            
            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                next_pos not in obstacles and next_pos not in visited):
                queue.append((next_pos, path + [d]))
                visited.add(next_pos)
    
    return []  # No valid path found
   

pass

# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    from collections import deque

def dfs(start, goal, obstacles, rows, cols):
    """Depth-First Search for a path in a grid."""
    stack = [(start, [])]  # (current_position, path_taken)
    visited = set()
    visited.add(start)
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    
    while stack:
        (current, path) = stack.pop()
        
        if current == goal:
            return path  # Return the path to the food
        
        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])
            
            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                next_pos not in obstacles and next_pos not in visited):
                stack.append((next_pos, path + [d]))
                visited.add(next_pos)
    
    return []  # No valid path found

    
pass

# Iterative Deepening Search (IDS Algorithm
def ids(start, goal, obstacles, rows, cols, max_depth=50):
    """Iterative Deepening Search for a path in a grid."""
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    
    def dls(current, path, depth, visited):
        """Depth-Limited Search (DLS) helper function."""
        if depth < 0:
            return []
        if current == goal:
            return path
        
        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])
            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                next_pos not in obstacles and next_pos not in visited):
                visited.add(next_pos)
                result = dls(next_pos, path + [d], depth - 1, visited)
                if result:
                    return result
                visited.remove(next_pos)  # Backtrack
        return []
    
    # Iteratively increase depth limit
    for depth in range(max_depth):
        visited = set()
        visited.add(start)
        result = dls(start, [], depth, visited)
        if result:
            return result  # Return first found path
    
    return []  # No valid path found

pass

# Uniform Cost Search (UCS) Algorithm
def ucs(start, goal, obstacles, rows, cols):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    priority_queue = [(0, start, [])]  # (cost, current_position, path)
    visited = {}

    while priority_queue:
        cost, current, path = heappop(priority_queue)

        # If reached goal, return path
        if current == goal:
            return path

        # If already visited with lower cost, skip
        if current in visited and visited[current] <= cost:
            continue

        visited[current] = cost

        # Explore neighbors
        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])

            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and
                next_pos not in obstacles):

                new_cost = cost + 1  # Each move has cost 1
                heappush(priority_queue, (new_cost, next_pos, path + [d]))

    return []  # No valid path found
    pass

# Greedy Best First Search Algorithm
import heapq
import math

def greedy_bfs(start, goal, obstacles, rows, cols):
    """Greedy Best-First Search Algorithm for Snake AI."""
    
    def heuristic(pos):
        """Manhattan Distance Heuristic"""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    open_list = [(heuristic(start), start, [])]  # (heuristic, position, path)
    visited = set()

    while open_list:
        _, current, path = heapq.heappop(open_list)

        if current == goal:
            return path  # Return found path

        if current in visited:
            continue
        visited.add(current)

        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])
            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                next_pos not in obstacles and next_pos not in visited):
                heapq.heappush(open_list, (heuristic(next_pos), next_pos, path + [d]))

    return []  # No path found

    pass

# A* Search Algorithm
def a_star(start, goal, obstacles, rows, cols):
    """A* Search Algorithm for Snake AI."""
    
    def heuristic(pos):
        """Manhattan Distance Heuristic"""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    open_list = [(heuristic(start), 0, start, [])]  # (f, g, position, path)
    g_cost = {start: 0}
    visited = set()

    while open_list:
        _, g, current, path = heapq.heappop(open_list)

        if current == goal:
            return path  # Return found path

        if current in visited:
            continue
        visited.add(current)

        for d in directions:
            next_pos = (current[0] + d[0], current[1] + d[1])
            if (0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and 
                next_pos not in obstacles and next_pos not in visited):

                new_g = g + 1  # Cost to move
                if next_pos not in g_cost or new_g < g_cost[next_pos]:
                    g_cost[next_pos] = new_g
                    f = new_g + heuristic(next_pos)  # f(n) = g(n) + h(n)
                    heapq.heappush(open_list, (f, new_g, next_pos, path + [d]))

    return []  # No path found

    
    pass
