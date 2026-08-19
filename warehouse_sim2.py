import heapq
import itertools
import tkinter as tk
import random

# ─── Constants ────────────────────────────────────────────────────────────────
GRID_SIZE = 10
OBSTACLES = {
    (2, 1), (2, 2), (2, 3),
    (4, 5), (4, 6),
    (6, 3), (6, 4), (6, 5),
    (7, 7), (8, 7)
}
CELL_SIZE = 52
PADDING = 10
PANEL_W = 210

# Colors (dark theme)
C_BG        = "#0f1117"
C_PANEL     = "#1a1d27"
C_GRID_BG   = "#14171f"
C_BORDER    = "#2a2d3a"
C_FREE      = "#1e2130"
C_OBSTACLE  = "#3a3040"
C_VISITED   = "#1a2a3a"
C_PATH      = "#1e3020"
C_START     = "#1a2a3a"
C_GOAL      = "#3a1a1a"
C_FORKLIFT  = "#4fc3f7"
C_CONTAINER = "#e65100"
C_ACCENT    = "#4fc3f7"
C_ACCENT2   = "#81c784"
C_WARN      = "#ef9a9a"
C_TEXT      = "#e8eaf6"
C_TEXT_DIM  = "#6b7280"
C_YELLOW    = "#ffd54f"

PHASE_IDLE          = "idle"
PHASE_ROUTE_TO_PKG  = "route_to_pkg"
PHASE_PICKUP        = "pickup"
PHASE_ROUTE_TO_BAY  = "route_to_bay"
PHASE_DELIVERED     = "delivered"

PHASES_LABELS = ["IDLE", "ROUTE TO PKG", "PICKUP", "ROUTE TO BAY", "DELIVERED"]
PHASES_KEYS   = [PHASE_IDLE, PHASE_ROUTE_TO_PKG, PHASE_PICKUP, PHASE_ROUTE_TO_BAY, PHASE_DELIVERED]

# ─── State Globals ────────────────────────────────────────────────────────────
forklift_pos         = None  # Initial forklift start position (cyan dot)
container_pos        = None  # Package position (orange box)
bay_pos              = None  # Destination position (red cell)
place_mode           = "forklift"  # Current placement selection: "forklift", "container", "bay"

phase                = PHASE_IDLE
sim_leg              = 1
search_step          = 0
move_step            = 0
carrying             = False
forklift_highlight   = False
forklift_current_pos = None

expanded_nodes       = []
path_result          = []

leg1_path            = []
leg1_expanded_count  = 0
leg1_expanded_order  = []

leg2_path            = []
leg2_expanded_count  = 0
leg2_expanded_order  = []

# UI Global components
grid_canvas       = None
btn_run           = None
btn_set_forklift  = None
btn_set_container = None
btn_set_bay       = None
phase_bar         = []

# Global StringVars
g_v_expanded = None
g_v_cost     = None
g_v_nodes    = None
g_v_status   = None

# ─── A* Algorithm ─────────────────────────────────────────────────────────────
def astar(grid, start, goal):
    """
    Runs A* Search algorithm on the grid from start to goal.
    grid: 10x10 2D list where 1 is obstacle, 0 is free space.
    start: (x, y) tuple representing starting coordinates.
    goal: (x, y) tuple representing target coordinates.
    
    Returns (path, expanded_count, expand_order_list).
    """
    counter = itertools.count()
    open_set = []
    g_score = {start: 0}
    h0 = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heapq.heappush(open_set, (h0, next(counter), start))
    came_from = {}
    closed_set = set()
    expand_order = []
    expanded_count = 0
    
    while open_set:
        f, _, current = heapq.heappop(open_set)
        if current in closed_set:
            continue
        closed_set.add(current)
        expand_order.append(current)
        expanded_count += 1
        
        if current == goal:
            path = []
            cur = goal
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path, expanded_count, expand_order
            
        cx, cy = current
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if grid[ny][nx] == 1:
                    continue
                nb = (nx, ny)
                tg = g_score[current] + 1
                if tg < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tg
                    h = abs(nx - goal[0]) + abs(ny - goal[1])
                    heapq.heappush(open_set, (tg + h, next(counter), nb))
                    
    return None, expanded_count, expand_order

# ─── Grid Drawing ─────────────────────────────────────────────────────────────
def cell_coords(x, y):
    x1 = PADDING + x * CELL_SIZE
    y1 = PADDING + y * CELL_SIZE
    return x1, y1, x1 + CELL_SIZE, y1 + CELL_SIZE

def cell_center(x, y):
    x1, y1, x2, y2 = cell_coords(x, y)
    return (x1 + x2) // 2, (y1 + y2) // 2

def draw_base_grid(canvas):
    canvas.delete("all")
    canvas_w = PADDING * 2 + GRID_SIZE * CELL_SIZE
    canvas_h = PADDING * 2 + GRID_SIZE * CELL_SIZE
    canvas.create_rectangle(0, 0, canvas_w, canvas_h, fill=C_GRID_BG, outline="")

    # Determine path and visited states to draw
    path_set = set()
    if sim_leg == 1:
        if phase != PHASE_ROUTE_TO_PKG or search_step >= len(leg1_expanded_order):
            path_set = set(leg1_path)
        visited_set = set(leg1_expanded_order[:search_step])
    else:
        if phase != PHASE_ROUTE_TO_BAY or search_step >= len(leg2_expanded_order):
            path_set = set(leg2_path)
        visited_set = set(leg2_expanded_order[:search_step])

    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            x1, y1, x2, y2 = cell_coords(x, y)
            
            if cell == forklift_pos:
                # Forklift start marker: cyan border, dim fill
                fill, ol, w = C_START, C_FORKLIFT, 2
            elif cell == container_pos and not carrying:
                # Container: orange filled cell
                fill, ol, w = C_CONTAINER, C_CONTAINER, 1
            elif cell == bay_pos:
                # Bay: dark red cell with light red border
                fill, ol, w = C_GOAL, C_WARN, 2
            elif cell in OBSTACLES:
                fill, ol, w = C_OBSTACLE, "#5a4060", 1
            elif cell in path_set:
                fill, ol, w = C_PATH, C_BORDER, 1
            elif cell in visited_set:
                fill, ol, w = C_VISITED, C_BORDER, 1
            else:
                fill, ol, w = C_FREE, C_BORDER, 1
                
            canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline=ol, width=w)

    # Draw shelf details
    for (ox, oy) in OBSTACLES:
        x1, y1, x2, y2 = cell_coords(ox, oy)
        mx = (x1 + x2) // 2
        for row in [y1 + 10, y1 + 26, y1 + 42]:
            canvas.create_line(x1 + 6, row, x2 - 6, row, fill="#7a5080", width=1)
        canvas.create_line(mx, y1 + 6, mx, y2 - 6, fill="#7a5080", width=1)

    # Draw START text label inside forklift start cell
    if forklift_pos is not None:
        cx, cy = cell_center(*forklift_pos)
        canvas.create_text(cx, cy - 5, text="▶", fill=C_ACCENT, font=("Courier", 11, "bold"))
        canvas.create_text(cx, cy + 8, text="START", fill=C_ACCENT, font=("Courier", 6, "bold"))

    # Draw container box icon and text inside container cell
    if container_pos is not None and not carrying:
        cx, cy = cell_center(*container_pos)
        canvas.create_text(cx, cy, text="■", fill="#ffffff", font=("Courier", 12, "bold"))

    # Draw BAY text label inside goal cell
    if bay_pos is not None:
        cx, cy = cell_center(*bay_pos)
        canvas.create_text(cx, cy - 5, text="◉", fill=C_WARN, font=("Courier", 11, "bold"))
        canvas.create_text(cx, cy + 8, text="BAY", fill=C_WARN, font=("Courier", 6, "bold"))

    # Overlay green dots for path cells (excluding key targets)
    if path_set:
        for px, py in path_set:
            if (px, py) in (forklift_pos, container_pos, bay_pos):
                continue
            cx2, cy2 = cell_center(px, py)
            canvas.create_oval(cx2 - 3, cy2 - 3, cx2 + 3, cy2 + 3, fill=C_ACCENT2, outline="")

    # Render forklift if active
    if forklift_current_pos is not None:
        draw_forklift(canvas, forklift_current_pos[0], forklift_current_pos[1], carrying, forklift_highlight)

def draw_forklift(canvas, x, y, carrying, highlight=False):
    x1, y1, x2, y2 = cell_coords(x, y)
    pad = 8
    col = "#ffffff" if highlight else C_FORKLIFT
    canvas.create_rectangle(
        x1 + pad, y1 + pad, x2 - pad, y2 - pad,
        fill=col, outline="#b3e5fc", width=2
    )
    cx, cy = cell_center(x, y)
    canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=C_BG, outline="")
    
    # Carrying indicator: small orange square drawn on top-right corner of forklift rect
    if carrying:
        rx2 = x2 - pad
        ry1 = y1 + pad
        canvas.create_rectangle(
            rx2 - 6, ry1, rx2 + 2, ry1 + 8,
            fill="#e65100", outline="#ffffff", width=1
        )

# ─── Visual Pipeline ──────────────────────────────────────────────────────────
C_FUTURE_DOT    = "#4a4e5d"
C_FUTURE_LBL    = "#555865"
C_COMPLETED_LBL = "#888c9d"

def update_phase_bar():
    idx = PHASES_KEYS.index(phase) if phase in PHASES_KEYS else 0
    for i, (dot, lbl) in enumerate(phase_bar):
        if i < idx:
            dot.config(fg=C_ACCENT2)  # green
            lbl.config(fg=C_COMPLETED_LBL)
        elif i == idx:
            dot.config(fg=C_ACCENT)   # cyan
            lbl.config(fg=C_TEXT)     # bright text
        else:
            dot.config(fg=C_FUTURE_DOT)  # dim
            lbl.config(fg=C_FUTURE_LBL)

def select_place_mode(mode):
    global place_mode
    place_mode = mode
    update_placement_buttons()

def update_placement_buttons():
    if place_mode == "forklift":
        btn_set_forklift.config(bg=C_FORKLIFT, fg=C_BG, activebackground=C_FORKLIFT, activeforeground=C_BG)
        btn_set_container.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
        btn_set_bay.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
    elif place_mode == "container":
        btn_set_forklift.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
        btn_set_container.config(bg=C_CONTAINER, fg="#ffffff", activebackground=C_CONTAINER, activeforeground="#ffffff")
        btn_set_bay.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
    elif place_mode == "bay":
        btn_set_forklift.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
        btn_set_container.config(bg="#2a2d3a", fg=C_TEXT_DIM, activebackground="#2a2d3a", activeforeground=C_TEXT_DIM)
        btn_set_bay.config(bg=C_WARN, fg=C_BG, activebackground=C_WARN, activeforeground=C_BG)

def disable_placement_buttons():
    btn_set_forklift.config(state="disabled")
    btn_set_container.config(state="disabled")
    btn_set_bay.config(state="disabled")

def enable_placement_buttons():
    btn_set_forklift.config(state="normal")
    btn_set_container.config(state="normal")
    btn_set_bay.config(state="normal")
    update_placement_buttons()

# ─── Animation Ticks ──────────────────────────────────────────────────────────
def tick_search(canvas, root, v_expanded, v_status):
    global search_step, phase
    
    if sim_leg == 1:
        nodes = leg1_expanded_order
    else:
        nodes = leg2_expanded_order
        
    if search_step < len(nodes):
        search_step += 1
        v_expanded.set(str(search_step))
        v_status.set(f"Expanding {search_step}/{len(nodes)}")
        draw_base_grid(canvas)
        update_phase_bar()
        speed = max(10, 80 - search_step)
        root.after(speed, lambda: tick_search(canvas, root, v_expanded, v_status))
    else:
        if sim_leg == 1:
            v_status.set(f"Leg 1 path found: {len(leg1_path)-1} steps")
            draw_base_grid(canvas)
            root.after(500, lambda: start_move_leg1(canvas, root, v_status))
        else:
            v_status.set(f"Leg 2 path found: {len(leg2_path)-1} steps")
            draw_base_grid(canvas)
            root.after(500, lambda: start_move_leg2(canvas, root, v_status))

def tick_move(canvas, root, v_status):
    global move_step, forklift_current_pos, forklift_highlight
    
    if sim_leg == 1:
        path = leg1_path
        status_label = "Navigating to package"
    else:
        path = leg2_path
        status_label = "Navigating to bay"
        
    if move_step < len(path):
        forklift_current_pos = path[move_step]
        
        if move_step == len(path) - 1:
            if sim_leg == 1:
                # Transition to Phase 2: PICKUP
                trigger_pickup(canvas, root, v_status)
            else:
                # Transition to Phase 4: DELIVER
                forklift_highlight = True
                v_status.set("DELIVERING...")
                draw_base_grid(canvas)
                root.after(400, lambda: finalize_mission_delivered(canvas, root, v_status))
        else:
            v_status.set(f"{status_label}... Step {move_step+1}/{len(path)}")
            draw_base_grid(canvas)
            move_step += 1
            root.after(180, lambda: tick_move(canvas, root, v_status))

def start_move_leg1(canvas, root, v_status):
    global move_step, forklift_current_pos
    move_step = 0
    forklift_current_pos = leg1_path[0]
    tick_move(canvas, root, v_status)

def start_move_leg2(canvas, root, v_status):
    global move_step, forklift_current_pos
    move_step = 0
    forklift_current_pos = leg2_path[0]
    tick_move(canvas, root, v_status)

def trigger_pickup(canvas, root, v_status):
    global phase, carrying, forklift_current_pos
    phase = PHASE_PICKUP
    carrying = True
    forklift_current_pos = container_pos
    v_status.set("PICKING UP PACKAGE...")
    update_phase_bar()
    draw_base_grid(canvas)
    
    # 600ms pickup pause
    root.after(600, lambda: start_leg2_search(canvas, root, v_status))

def start_leg2_search(canvas, root, v_status):
    global phase, sim_leg, search_step, expanded_nodes, path_result, forklift_current_pos
    phase = PHASE_ROUTE_TO_BAY
    sim_leg = 2
    search_step = 0
    forklift_current_pos = container_pos
    expanded_nodes = leg2_expanded_order
    path_result = leg2_path
    v_status.set("Searching path to bay...")
    update_phase_bar()
    draw_base_grid(canvas)
    
    # Run Leg 2 search animation tick
    root.after(200, lambda: tick_search(canvas, root, g_v_expanded, v_status))

def finalize_mission_delivered(canvas, root, v_status):
    global phase, carrying, forklift_highlight, forklift_current_pos
    phase = PHASE_DELIVERED
    carrying = False  # package delivered!
    forklift_highlight = False
    forklift_current_pos = bay_pos
    v_status.set("DELIVERED ✓")
    update_phase_bar()
    draw_base_grid(canvas)
    
    # Re-enable UI buttons
    btn_run.config(state="normal", text="▶  RUN AGAIN")
    enable_placement_buttons()

# ─── Canvas Click Handler ─────────────────────────────────────────────────────
def on_canvas_click(event):
    global forklift_pos, container_pos, bay_pos
    
    # Clicks are ignored during active simulation run
    if phase not in (PHASE_IDLE, PHASE_DELIVERED):
        return
        
    cx = (event.x - PADDING) // CELL_SIZE
    cy = (event.y - PADDING) // CELL_SIZE
    
    if not (0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE):
        return
        
    if (cx, cy) in OBSTACLES:
        g_v_status.set("Cannot place on shelf obstacles!")
        return
        
    # Clear coordinate if occupied by another element
    if (cx, cy) == forklift_pos:
        forklift_pos = None
    if (cx, cy) == container_pos:
        container_pos = None
    if (cx, cy) == bay_pos:
        bay_pos = None
        
    # Assign position based on place mode
    if place_mode == "forklift":
        forklift_pos = (cx, cy)
    elif place_mode == "container":
        container_pos = (cx, cy)
    elif place_mode == "bay":
        bay_pos = (cx, cy)
        
    # Recalculate which items are left to place
    to_place = []
    if forklift_pos is None: to_place.append("Forklift")
    if container_pos is None: to_place.append("Container")
    if bay_pos is None: to_place.append("Bay")
    
    if to_place:
        g_v_status.set(f"Place: {', '.join(to_place)}")
        btn_run.config(state="disabled")
    else:
        g_v_status.set("Ready to run mission!")
        btn_run.config(state="normal")
        
    draw_base_grid(grid_canvas)

# ─── Run Simulation ───────────────────────────────────────────────────────────
def run_simulation(canvas, root, v_expanded, v_cost, v_nodes, v_status):
    global expanded_nodes, path_result, phase, search_step, move_step, sim_leg, carrying, forklift_current_pos, forklift_highlight
    global leg1_path, leg1_expanded_count, leg1_expanded_order
    global leg2_path, leg2_expanded_count, leg2_expanded_order

    if forklift_pos is None or container_pos is None or bay_pos is None:
        return

    phase = PHASE_ROUTE_TO_PKG
    sim_leg = 1
    search_step = 0
    move_step = 0
    carrying = False
    forklift_highlight = False
    forklift_current_pos = forklift_pos

    v_expanded.set("0")
    v_cost.set("—")
    v_nodes.set("—")
    v_status.set("Running mission A*...")
    btn_run.config(state="disabled", text="RUNNING...")
    disable_placement_buttons()

    # Construct obstacle grid representation
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
    for ox, oy in OBSTACLES:
        grid[oy][ox] = 1

    # Solve Leg 1: forklift start to container
    path1, count1, order1 = astar(grid, forklift_pos, container_pos)
    # Solve Leg 2: container to bay
    path2, count2, order2 = astar(grid, container_pos, bay_pos)

    if not path1:
        v_status.set("No path to container!")
        btn_run.config(state="normal", text="▶  RUN SIMULATION")
        enable_placement_buttons()
        return
    if not path2:
        v_status.set("No path to bay!")
        btn_run.config(state="normal", text="▶  RUN SIMULATION")
        enable_placement_buttons()
        return

    # Cache leg data
    leg1_path = path1
    leg1_expanded_count = count1
    leg1_expanded_order = order1

    leg2_path = path2
    leg2_expanded_count = count2
    leg2_expanded_order = order2

    # Output details to console
    leg1_cost = len(path1) - 1
    leg2_cost = len(path2) - 1
    full_path = path1 + path2[1:]
    print(f"Leg 1 — cost: {leg1_cost}, expanded: {count1}")
    print(f"Leg 2 — cost: {leg2_cost}, expanded: {count2}")
    print(f"Full path: {full_path}")

    # Set initial metrics
    v_cost.set(str(leg1_cost + leg2_cost))
    v_nodes.set(str(count1 + count2))

    # Initialize leg 1 search animation properties
    expanded_nodes = leg1_expanded_order
    path_result = leg1_path
    
    update_phase_bar()
    draw_base_grid(canvas)
    root.after(200, lambda: tick_search(canvas, root, v_expanded, v_status))

# ─── Main Application Builder ─────────────────────────────────────────────────
def main():
    global grid_canvas, btn_run, btn_set_forklift, btn_set_container, btn_set_bay, phase_bar
    global g_v_expanded, g_v_cost, g_v_nodes, g_v_status

    root = tk.Tk()
    root.title("Warehouse Mission")
    root.configure(bg=C_BG)
    root.resizable(False, False)

    canvas_size = PADDING * 2 + GRID_SIZE * CELL_SIZE  # 540px
    total_w = canvas_size + PANEL_W

    root.geometry(f"{total_w}x{canvas_size}+{(root.winfo_screenwidth()-total_w)//2}+{(root.winfo_screenheight()-canvas_size)//2}")

    # Left Side Canvas Frame
    left = tk.Frame(root, bg=C_BG)
    left.pack(side="left", fill="both")
    grid_canvas = tk.Canvas(
        left, width=canvas_size, height=canvas_size,
        bg=C_GRID_BG, highlightthickness=0
    )
    grid_canvas.pack(fill="both", expand=True)
    grid_canvas.bind("<Button-1>", on_canvas_click)

    # Right Side Panel Frame
    right_outer = tk.Frame(root, bg=C_PANEL, width=PANEL_W)
    right_outer.pack(side="right", fill="both")
    right_outer.pack_propagate(False)

    right = tk.Frame(right_outer, bg=C_PANEL)
    right.pack(fill="both", expand=True)

    def sep():
        tk.Frame(right, bg=C_BORDER, height=1).pack(side="top", fill="x", padx=12, pady=3)

    # Pinned Run Button packed first with side="bottom" and expand=True
    btn_run = tk.Button(
        right, text="▶  RUN SIMULATION",
        bg=C_ACCENT, fg=C_BG,
        activebackground="#81d4fa", activeforeground=C_BG,
        font=("Courier", 9, "bold"),
        relief="flat", cursor="hand2",
        padx=6, pady=6,
        command=lambda: run_simulation(
            grid_canvas, root, g_v_expanded, g_v_cost, g_v_nodes, g_v_status)
    )
    btn_run.pack(side="bottom", fill="x", padx=12, pady=(0, 10), expand=True)
    btn_run.config(state="disabled")

    # Title
    title_frame = tk.Frame(right, bg=C_PANEL)
    title_frame.pack(side="top", fill="x", pady=(8, 0))
    tk.Label(title_frame, text="WAREHOUSE MISSION", bg=C_PANEL, fg=C_ACCENT,
             font=("Courier", 10, "bold")).pack()
    
    # Placement Mode Buttons
    btn_frame = tk.Frame(right, bg=C_PANEL)
    btn_frame.pack(side="top", fill="x", padx=12, pady=(5, 0))
    
    btn_set_forklift = tk.Button(
        btn_frame, text="FORKLIFT", font=("Courier", 7, "bold"),
        relief="flat", cursor="hand2", bg="#2a2d3a", fg=C_TEXT_DIM,
        padx=4, pady=4, command=lambda: select_place_mode("forklift")
    )
    btn_set_forklift.pack(side="left", expand=True, fill="x", padx=1)
    
    btn_set_container = tk.Button(
        btn_frame, text="PACKAGE", font=("Courier", 7, "bold"),
        relief="flat", cursor="hand2", bg="#2a2d3a", fg=C_TEXT_DIM,
        padx=4, pady=4, command=lambda: select_place_mode("container")
    )
    btn_set_container.pack(side="left", expand=True, fill="x", padx=1)
    
    btn_set_bay = tk.Button(
        btn_frame, text="BAY", font=("Courier", 7, "bold"),
        relief="flat", cursor="hand2", bg="#2a2d3a", fg=C_TEXT_DIM,
        padx=4, pady=4, command=lambda: select_place_mode("bay")
    )
    btn_set_bay.pack(side="left", expand=True, fill="x", padx=1)

    sep()

    # Pipeline
    pipeline_frame = tk.Frame(right, bg=C_PANEL)
    pipeline_frame.pack(side="top", fill="x", padx=12)
    tk.Label(pipeline_frame, text="PIPELINE", bg=C_PANEL, fg=C_TEXT_DIM,
             font=("Courier", 7, "bold")).pack(anchor="w")
    pipe = tk.Frame(pipeline_frame, bg=C_PANEL)
    pipe.pack(side="top", fill="x", pady=(1, 2))
    phase_bar = []
    for lbl in PHASES_LABELS:
        row = tk.Frame(pipe, bg=C_PANEL)
        row.pack(side="top", fill="x", pady=0)
        dot = tk.Label(row, text="●", bg=C_PANEL, fg=C_BORDER, font=("Courier", 8))
        dot.pack(side="left")
        ltxt = tk.Label(row, text=lbl, bg=C_PANEL, fg=C_TEXT_DIM, font=("Courier", 7))
        ltxt.pack(side="left", padx=3)
        phase_bar.append((dot, ltxt))

    sep()

    # Metrics
    metrics_frame = tk.Frame(right, bg=C_PANEL)
    metrics_frame.pack(side="top", fill="x", padx=12)
    tk.Label(metrics_frame, text="METRICS", bg=C_PANEL, fg=C_TEXT_DIM,
             font=("Courier", 7, "bold")).pack(anchor="w")

    g_v_expanded = tk.StringVar(value="0")
    g_v_cost     = tk.StringVar(value="—")
    g_v_nodes    = tk.StringVar(value="—")
    g_v_status   = tk.StringVar(value="Place: Forklift, Container, Bay")

    def stat(label, var, color):
        f = tk.Frame(metrics_frame, bg=C_PANEL)
        f.pack(side="top", fill="x", pady=1)
        tk.Label(f, text=label, bg=C_PANEL, fg=C_TEXT_DIM, font=("Courier", 7)).pack(anchor="w")
        tk.Label(f, textvariable=var, bg=C_PANEL, fg=color, font=("Courier", 10, "bold"), wraplength=180, justify="left").pack(anchor="w")

    stat("NODES EXPANDED (live)", g_v_expanded, C_ACCENT)
    stat("TOTAL COST (both legs)", g_v_cost,     C_ACCENT2)
    stat("MISSION STATUS",        g_v_status,   C_TEXT)

    sep()

    # Legend
    legend_frame = tk.Frame(right, bg=C_PANEL)
    legend_frame.pack(side="top", fill="x", padx=12)
    tk.Label(legend_frame, text="LEGEND", bg=C_PANEL, fg=C_TEXT_DIM,
             font=("Courier", 7, "bold")).pack(anchor="w")
    leg = tk.Frame(legend_frame, bg=C_PANEL)
    leg.pack(side="top", fill="x", pady=(1, 2))

    def lrow(col, label, ol=None):
        f = tk.Frame(leg, bg=C_PANEL)
        f.pack(side="top", fill="x", pady=0)
        b = tk.Canvas(f, width=10, height=10, bg=C_PANEL, highlightthickness=0)
        b.pack(side="left", padx=(0, 6))
        b.create_rectangle(0, 0, 9, 9, fill=col, outline=ol or col)
        tk.Label(f, text=label, bg=C_PANEL, fg=C_TEXT_DIM, font=("Courier", 7)).pack(side="left")

    lrow(C_FORKLIFT,  "Forklift",   "#b3e5fc")
    lrow(C_CONTAINER, "Container",  C_CONTAINER)
    lrow(C_GOAL,      "Loading bay",C_WARN)
    lrow(C_OBSTACLE,  "Shelf",      "#7a5080")
    lrow(C_VISITED,   "Expanded",   C_BORDER)
    lrow(C_PATH,      "Path",       C_ACCENT2)

    update_placement_buttons()
    draw_base_grid(grid_canvas)
    update_phase_bar()
    root.mainloop()

if __name__ == "__main__":
    main()