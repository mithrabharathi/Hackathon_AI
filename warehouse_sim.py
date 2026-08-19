import heapq
import itertools
import tkinter as tk

# 1. Grid Constants and Obstacles
GRID_SIZE = 10
START = (0, 0)
GOAL = (9, 9)
OBSTACLES = {
    (2, 1), (2, 2), (2, 3),
    (4, 5), (4, 6),
    (6, 3), (6, 4), (6, 5),
    (7, 7), (8, 7)
}
CELL_SIZE = 50

# Global tracking for visited/expanded nodes
expanded_nodes = set()
forklift_id = None

# 2. A* Search Algorithm
def astar(grid, start, goal):
    """
    Runs A* Search algorithm on the grid from start to goal.
    grid: 10x10 2D list where 1 is obstacle, 0 is free space.
    start: (x, y) tuple representing starting coordinates.
    goal: (x, y) tuple representing target coordinates.
    
    Returns (path, expanded_count) or (None, expanded_count) if no path is found.
    """
    global expanded_nodes
    expanded_nodes.clear()
    
    # Priority queue stores: (f_score, unique_counter, current_node)
    # unique_counter is used as a tie-breaker to prevent coordinate comparison errors
    counter = itertools.count()
    open_set = []
    
    g_score = {start: 0}
    h_start = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heapq.heappush(open_set, (h_start, next(counter), start))
    
    came_from = {}
    expanded_set = set()
    expanded_count = 0
    
    while open_set:
        f, _, current = heapq.heappop(open_set)
        
        # If we reached the goal cell
        if current == goal:
            # Reconstruct path
            path = []
            curr = goal
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            return path, expanded_count
        
        if current in expanded_set:
            continue
            
        expanded_set.add(current)
        expanded_nodes.add(current)
        expanded_count += 1
        
        cx, cy = current
        
        # Neighbors: Up, Down, Left, Right
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = cx + dx, cy + dy
            
            # Check grid boundaries
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                # Check if cell is not an obstacle
                if grid[ny][nx] == 1:
                    continue
                    
                neighbor = (nx, ny)
                tentative_g_score = g_score[current] + 1
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    h = abs(nx - goal[0]) + abs(ny - goal[1])
                    f_score = tentative_g_score + h
                    heapq.heappush(open_set, (f_score, next(counter), neighbor))
                    
    return None, expanded_count

# 3. Draw Grid State
def draw_grid(canvas, path, visited):
    """
    Renders the static grid state on the canvas.
    canvas: tkinter.Canvas object.
    path: list of (x, y) coordinates representing optimal path.
    visited: set of (x, y) coordinates representing expanded nodes.
    """
    path_set = set(path) if path else set()
    
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            
            # Determine color
            if cell == START:
                color = "#2ecc71"  # Green
            elif cell == GOAL:
                color = "#e74c3c"  # Red
            elif cell in OBSTACLES:
                color = "#333333"  # Dark Gray
            elif cell in path_set:
                color = "#f1c40f"  # Yellow
            elif cell in visited:
                color = "#d6eaf8"  # Light Blue
            else:
                color = "#ffffff"  # White
                
            x1 = x * CELL_SIZE
            y1 = y * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE
            
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="#cccccc", width=1)
            
    # Draw Start and Goal text labels
    canvas.create_text(
        START[0] * CELL_SIZE + CELL_SIZE / 2,
        START[1] * CELL_SIZE + CELL_SIZE / 2,
        text="START",
        font=("Arial", 9, "bold"),
        fill="#000000"
    )
    canvas.create_text(
        GOAL[0] * CELL_SIZE + CELL_SIZE / 2,
        GOAL[1] * CELL_SIZE + CELL_SIZE / 2,
        text="BAY",
        font=("Arial", 9, "bold"),
        fill="#ffffff"
    )

# 4. Animate Path
def animate_path(canvas, path, step=0):
    """
    Animate the forklift icon moving cell by cell along the path.
    canvas: tkinter.Canvas object.
    path: list of (x, y) coordinates representing optimal path.
    step: current index of the cell in the path.
    """
    global forklift_id
    if path is None or step >= len(path):
        return
        
    x, y = path[step]
    x1 = x * CELL_SIZE + 10
    y1 = y * CELL_SIZE + 10
    x2 = (x + 1) * CELL_SIZE - 10
    y2 = (y + 1) * CELL_SIZE - 10
    
    if forklift_id is None:
        # Draw forklift as a filled blue square inside the cell
        forklift_id = canvas.create_rectangle(
            x1, y1, x2, y2,
            fill="#1f77b4",
            outline="#0b5394",
            width=2
        )
    else:
        # Update coordinates to move forklift
        canvas.coords(forklift_id, x1, y1, x2, y2)
        # Raise forklift icon to the top of the rendering stack
        canvas.tag_raise(forklift_id)
        
    # Recursive tkinter after-loop call with 200ms delay
    canvas.after(200, lambda: animate_path(canvas, path, step + 1))

# 5. Main Execution Function
def main():
    # Construct grid
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for x, y in OBSTACLES:
        grid[y][x] = 1
        
    # Run A* Pathfinding
    path, expanded_count = astar(grid, START, GOAL)
    
    # Print results to console
    if path:
        print(f"Total path cost: {len(path) - 1} steps")
        print(f"Nodes expanded: {expanded_count}")
        print(f"Path: {path}")
    else:
        print("No path found.")
        print(f"Nodes expanded: {expanded_count}")
        
    # Build Tkinter window
    root = tk.Tk()
    root.title("Warehouse Forklift Simulation")
    root.resizable(False, False)
    
    # Set window to center of screen
    window_size = GRID_SIZE * CELL_SIZE
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_size) // 2
    y = (screen_height - window_size) // 2
    root.geometry(f"{window_size}x{window_size}+{x}+{y}")
    
    canvas = tk.Canvas(root, width=window_size, height=window_size, bg="#ffffff", highlightthickness=0)
    canvas.pack()
    
    # Draw static grid state
    draw_grid(canvas, path, expanded_nodes)
    
    # Start animation of forklift
    if path:
        # Delay the animation start slightly (e.g. 500ms) for visual comfort
        canvas.after(500, lambda: animate_path(canvas, path))
        
    root.mainloop()

if __name__ == "__main__":
    main()
