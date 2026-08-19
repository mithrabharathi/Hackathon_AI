"""
Warehouse Logistics Agent — Track 1 (Unit 2: Informed Search)
----------------------------------------------------------------
Autonomous forklift that plans an optimal route from a pickup point
(START) to a loading bay (GOAL) in a grid warehouse using A* Search
with the Manhattan Distance heuristic:

    h(n) = |x1 - x2| + |y1 - y2|

The window shows:
  * a live "decision-making" replay of every node A* expands
    (with its g, h, f values) streamed into a log panel in real time
  * the optimal path highlighted once the search finishes
  * a forklift sprite (body / cab / forks / wheels) that drives the
    path, picks a package up at START and drops it at the GOAL bay
  * final performance metrics: path cost, nodes expanded, time taken
"""

import heapq
import itertools
import time
import tkinter as tk

# ============================================================
# 1. Grid Constants and Obstacles
# ============================================================
GRID_SIZE = 10
START = (0, 0)
GOAL = (9, 9)
OBSTACLES = {
    (2, 1), (2, 2), (2, 3),
    (4, 5), (4, 6),
    (6, 3), (6, 4), (6, 5),
    (7, 7), (8, 7)
}
CELL_SIZE = 52
GRID_PIXELS = GRID_SIZE * CELL_SIZE
LEGEND_HEIGHT = 30
CANVAS_HEIGHT = GRID_PIXELS + LEGEND_HEIGHT
PANEL_WIDTH = 360

# Timing (ms) for the two animation phases
EXPANSION_STEP_DELAY = 18     # speed of the "thinking" replay
MOVE_STEP_DELAY = 220         # speed of the forklift driving

# ============================================================
# 2. Color Palette (industrial warehouse theme)
# ============================================================
COLOR_BG            = "#0f1620"
COLOR_HEADER_BG     = "#16202c"
COLOR_FLOOR_A       = "#eef2f6"
COLOR_FLOOR_B       = "#e4eaf1"
COLOR_GRID_LINE     = "#c3cdd8"
COLOR_SHELF         = "#6b4f36"
COLOR_SHELF_DARK    = "#4a3423"
COLOR_SHELF_SLAT    = "#8a6a48"
COLOR_START_PAD     = "#27ae60"
COLOR_START_EDGE    = "#1e8449"
COLOR_GOAL_PAD      = "#e74c3c"
COLOR_GOAL_EDGE     = "#b03a2e"
COLOR_VISITED       = "#bfe0f7"
COLOR_VISITED_EDGE  = "#8fc7ec"
COLOR_PATH          = "#f4d35e"
COLOR_PATH_EDGE     = "#d8ab1f"
COLOR_FORK_BODY     = "#f5a623"
COLOR_FORK_BODY_DK  = "#c97e0e"
COLOR_FORK_CAB      = "#2c3e50"
COLOR_FORK_PRONG    = "#95a5a6"
COLOR_WHEEL         = "#1b1b1b"
COLOR_PACKAGE       = "#a9713f"
COLOR_PACKAGE_EDGE  = "#6f4423"
COLOR_PACKAGE_BAND  = "#e8d9c4"
COLOR_PANEL_BG      = "#131c27"
COLOR_LOG_BG        = "#0b1119"
COLOR_LOG_TEXT      = "#8ee08e"
COLOR_LOG_HEADER    = "#5dade2"
COLOR_TEXT_LIGHT    = "#ecf0f1"
COLOR_TEXT_DIM      = "#8ea0b3"
COLOR_ACCENT        = "#3498db"
COLOR_METRIC_BG     = "#1c2833"


# ============================================================
# 3. A* Search Algorithm (with a full decision trace)
# ============================================================
def astar(grid, start, goal):
    """
    Runs A* Search on the grid from start to goal using the
    Manhattan distance heuristic h(n) = |x1-x2| + |y1-y2|.

    Returns:
        path            - list of (x, y) cells from start to goal (or None)
        expanded_count  - number of nodes expanded (popped & processed)
        trace           - ordered list of dicts describing every expansion:
                           {node, g, h, f, order}. This is what powers the
                           real-time "decision log" panel in the UI.
    """
    counter = itertools.count()
    open_set = []

    g_score = {start: 0}
    h_start = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heapq.heappush(open_set, (h_start, next(counter), start))

    came_from = {}
    expanded_set = set()
    expanded_count = 0
    trace = []

    while open_set:
        f, _, current = heapq.heappop(open_set)

        if current in expanded_set:
            continue

        expanded_set.add(current)
        expanded_count += 1
        g_here = g_score[current]
        h_here = f - g_here
        trace.append({
            "node": current, "g": g_here, "h": h_here, "f": f,
            "order": expanded_count
        })

        # Goal test
        if current == goal:
            path = []
            curr = goal
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            return path, expanded_count, trace

        cx, cy = current
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if grid[ny][nx] == 1:
                    continue
                neighbor = (nx, ny)
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = abs(nx - goal[0]) + abs(ny - goal[1])
                    heapq.heappush(open_set, (tentative_g + h, next(counter), neighbor))

    return None, expanded_count, trace


# ============================================================
# 4. GUI / Animation
# ============================================================
class ForkliftSim:
    def __init__(self, root, grid, path, expanded_count, trace, search_time):
        self.root = root
        self.grid = grid
        self.path = path
        self.expanded_count = expanded_count
        self.trace = trace
        self.search_time = search_time

        self.forklift_dir = (1, 0)         # facing right by default
        self.carrying_package = False
        self.package_id = []               # canvas ids for the crate sprite
        self.forklift_ids = []             # canvas ids for the forklift sprite

        self._build_layout()
        self._draw_static_grid()
        self._draw_start_goal()
        self._draw_legend()
        self._draw_package(START, on_forklift=False)
        self._draw_forklift(START, (1, 0))

        # Kick off the "decision-making" replay, then the delivery run
        self.root.after(500, self._replay_expansions, 0)

    # -------------------- layout scaffolding --------------------
    def _build_layout(self):
        self.root.configure(bg=COLOR_BG)

        header = tk.Frame(self.root, bg=COLOR_HEADER_BG, height=56)
        header.pack(side="top", fill="x")
        tk.Label(
            header, text="🚜  Warehouse Forklift Agent — A* Pathfinding",
            font=("Segoe UI", 14, "bold"), bg=COLOR_HEADER_BG, fg=COLOR_TEXT_LIGHT
        ).pack(side="left", padx=16, pady=12)
        self.status_var = tk.StringVar(value="Planning route…")
        tk.Label(
            header, textvariable=self.status_var, font=("Segoe UI", 10, "italic"),
            bg=COLOR_HEADER_BG, fg=COLOR_ACCENT
        ).pack(side="right", padx=16)

        body = tk.Frame(self.root, bg=COLOR_BG)
        body.pack(side="top", fill="both", expand=True)

        # ---- left: canvas ----
        canvas_frame = tk.Frame(body, bg=COLOR_BG, padx=14, pady=14)
        canvas_frame.pack(side="left")
        self.canvas = tk.Canvas(
            canvas_frame, width=GRID_PIXELS, height=CANVAS_HEIGHT,
            bg=COLOR_FLOOR_A, highlightthickness=2, highlightbackground="#2c3e50"
        )
        self.canvas.pack()

        # ---- right: metrics + live log ----
        panel = tk.Frame(body, bg=COLOR_PANEL_BG, width=PANEL_WIDTH)
        panel.pack(side="right", fill="both", expand=True, padx=(0, 14), pady=14)
        panel.pack_propagate(False)

        tk.Label(
            panel, text="Live Decision Log", font=("Segoe UI", 11, "bold"),
            bg=COLOR_PANEL_BG, fg=COLOR_LOG_HEADER
        ).pack(anchor="w", padx=12, pady=(10, 4))

        log_frame = tk.Frame(panel, bg=COLOR_PANEL_BG)
        log_frame.pack(fill="both", expand=True, padx=12)
        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side="right", fill="y")
        self.log_text = tk.Text(
            log_frame, bg=COLOR_LOG_BG, fg=COLOR_LOG_TEXT, insertbackground=COLOR_LOG_TEXT,
            font=("Consolas", 9), wrap="none", relief="flat",
            yscrollcommand=scrollbar.set
        )
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)
        self.log_text.tag_config("goal", foreground="#f4d35e")
        self.log_text.tag_config("move", foreground="#5dade2")
        self.log_text.tag_config("event", foreground="#e67e22")

        # ---- metrics strip ----
        metrics = tk.Frame(panel, bg=COLOR_METRIC_BG)
        metrics.pack(fill="x", padx=12, pady=10)
        self.cost_var = tk.StringVar(value="Path Cost: —")
        self.nodes_var = tk.StringVar(value="Nodes Expanded: —")
        self.time_var = tk.StringVar(value="Time Taken: —")
        for var in (self.cost_var, self.nodes_var, self.time_var):
            tk.Label(
                metrics, textvariable=var, font=("Segoe UI", 10, "bold"),
                bg=COLOR_METRIC_BG, fg=COLOR_TEXT_LIGHT, anchor="w"
            ).pack(fill="x", padx=10, pady=4)

        # ---- replay button ----
        self.replay_btn = tk.Button(
            panel, text="🔁 Replay Simulation", font=("Segoe UI", 10, "bold"),
            bg=COLOR_ACCENT, fg="white", relief="flat", activebackground="#2980b9",
            command=self.restart, state="disabled"
        )
        self.replay_btn.pack(fill="x", padx=12, pady=(0, 12))

    def _log(self, message, tag=None):
        self.log_text.insert("end", message + "\n", tag)
        self.log_text.see("end")

    # -------------------- static drawing --------------------
    def _draw_static_grid(self):
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                base = COLOR_FLOOR_A if (x + y) % 2 == 0 else COLOR_FLOOR_B
                x1, y1 = x * CELL_SIZE, y * CELL_SIZE
                x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE
                self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill=base, outline=COLOR_GRID_LINE, width=1,
                    tags=(f"cell_{x}_{y}", "floor")
                )
        for (x, y) in OBSTACLES:
            self._draw_shelf(x, y)

    def _draw_shelf(self, x, y):
        x1, y1 = x * CELL_SIZE, y * CELL_SIZE
        x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE
        self.canvas.create_rectangle(x1 + 3, y1 + 3, x2 - 3, y2 - 3,
                                      fill=COLOR_SHELF, outline=COLOR_SHELF_DARK, width=2)
        # shelving slats for a "storage rack" look
        for i in range(1, 4):
            sy = y1 + i * (CELL_SIZE / 4)
            self.canvas.create_line(x1 + 5, sy, x2 - 5, sy, fill=COLOR_SHELF_SLAT, width=2)

    def _draw_start_goal(self):
        sx, sy = START
        gx, gy = GOAL
        x1, y1 = sx * CELL_SIZE, sy * CELL_SIZE
        self.canvas.create_rectangle(x1 + 4, y1 + 4, x1 + CELL_SIZE - 4, y1 + CELL_SIZE - 4,
                                      fill=COLOR_START_PAD, outline=COLOR_START_EDGE, width=2)
        self.canvas.create_text(x1 + CELL_SIZE / 2, y1 + CELL_SIZE / 2, text="PICKUP",
                                 font=("Segoe UI", 8, "bold"), fill="white")
        x1, y1 = gx * CELL_SIZE, gy * CELL_SIZE
        self.canvas.create_rectangle(x1 + 4, y1 + 4, x1 + CELL_SIZE - 4, y1 + CELL_SIZE - 4,
                                      fill=COLOR_GOAL_PAD, outline=COLOR_GOAL_EDGE, width=2)
        # loading bay "stripes"
        for i in range(4):
            lx = x1 + 8 + i * 10
            self.canvas.create_line(lx, y1 + CELL_SIZE - 6, lx + 5, y1 + 6,
                                     fill="white", width=2)
        self.canvas.create_text(x1 + CELL_SIZE / 2, y1 + CELL_SIZE / 2, text="BAY",
                                 font=("Segoe UI", 9, "bold"), fill="white")

    def _draw_legend(self):
        items = [
            (COLOR_START_PAD, "Pickup point"),
            (COLOR_GOAL_PAD, "Loading bay"),
            (COLOR_SHELF, "Shelf obstacle"),
            (COLOR_VISITED, "Expanded node"),
            (COLOR_PATH, "Optimal path"),
        ]
        x0, y0 = 8, GRID_PIXELS + LEGEND_HEIGHT / 2 + 4
        self.canvas.create_rectangle(0, GRID_PIXELS, GRID_PIXELS, CANVAS_HEIGHT,
                                      fill=COLOR_HEADER_BG, outline="")
        for i, (color, label) in enumerate(items):
            lx = x0 + i * (GRID_PIXELS / len(items))
            self.canvas.create_rectangle(lx, y0 - 6, lx + 12, y0 + 6, fill=color, outline="#333")
            self.canvas.create_text(lx + 16, y0, text=label, anchor="w",
                                     font=("Segoe UI", 7), fill=COLOR_TEXT_LIGHT)

    # -------------------- forklift / package sprites --------------------
    def _draw_package(self, cell, on_forklift):
        for cid in self.package_id:
            self.canvas.delete(cid)
        self.package_id = []
        x, y = cell
        cx = x * CELL_SIZE + CELL_SIZE / 2
        cy = y * CELL_SIZE + CELL_SIZE / 2
        if on_forklift:
            cy -= 14  # sits on top of the forklift cab
        size = 16
        crate = self.canvas.create_rectangle(
            cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2,
            fill=COLOR_PACKAGE, outline=COLOR_PACKAGE_EDGE, width=2, tags="package"
        )
        band_h = self.canvas.create_line(cx - size / 2, cy, cx + size / 2, cy,
                                          fill=COLOR_PACKAGE_BAND, width=2, tags="package")
        band_v = self.canvas.create_line(cx, cy - size / 2, cx, cy + size / 2,
                                          fill=COLOR_PACKAGE_BAND, width=2, tags="package")
        self.package_id = [crate, band_h, band_v]

    def _clear_package(self):
        for cid in self.package_id:
            self.canvas.delete(cid)
        self.package_id = []

    def _draw_forklift(self, cell, direction):
        for cid in self.forklift_ids:
            self.canvas.delete(cid)
        self.forklift_ids = []

        x, y = cell
        dx, dy = direction
        cx1 = x * CELL_SIZE + 8
        cy1 = y * CELL_SIZE + 10
        cx2 = x * CELL_SIZE + CELL_SIZE - 8
        cy2 = y * CELL_SIZE + CELL_SIZE - 10
        ids = []

        # wheels
        wy = cy2 - 2
        ids.append(self.canvas.create_oval(cx1 + 2, wy - 4, cx1 + 10, wy + 4, fill=COLOR_WHEEL))
        ids.append(self.canvas.create_oval(cx2 - 10, wy - 4, cx2 - 2, wy + 4, fill=COLOR_WHEEL))

        # body
        ids.append(self.canvas.create_rectangle(
            cx1, cy1 + 6, cx2 - 8, cy2 - 4, fill=COLOR_FORK_BODY, outline=COLOR_FORK_BODY_DK, width=2
        ))
        # cab
        ids.append(self.canvas.create_rectangle(
            cx1 + 4, cy1 - 4, cx1 + (cx2 - cx1) * 0.55, cy1 + 8,
            fill=COLOR_FORK_CAB, outline="#1b2631", width=1
        ))

        # forks — positioned on the side matching the direction of travel
        prong_len = 10
        if dx == 1:      # moving right -> forks point right
            fx = cx2 - 8
            ids.append(self.canvas.create_line(fx, cy1 + 12, fx + prong_len, cy1 + 12, fill=COLOR_FORK_PRONG, width=3))
            ids.append(self.canvas.create_line(fx, cy2 - 8, fx + prong_len, cy2 - 8, fill=COLOR_FORK_PRONG, width=3))
        elif dx == -1:    # moving left
            fx = cx1
            ids.append(self.canvas.create_line(fx, cy1 + 12, fx - prong_len, cy1 + 12, fill=COLOR_FORK_PRONG, width=3))
            ids.append(self.canvas.create_line(fx, cy2 - 8, fx - prong_len, cy2 - 8, fill=COLOR_FORK_PRONG, width=3))
        elif dy == -1:    # moving up
            fy = cy1 + 6
            ids.append(self.canvas.create_line(cx1 + 6, fy, cx1 + 6, fy - prong_len, fill=COLOR_FORK_PRONG, width=3))
            ids.append(self.canvas.create_line(cx2 - 14, fy, cx2 - 14, fy - prong_len, fill=COLOR_FORK_PRONG, width=3))
        else:             # moving down (default)
            fy = cy2 - 4
            ids.append(self.canvas.create_line(cx1 + 6, fy, cx1 + 6, fy + prong_len, fill=COLOR_FORK_PRONG, width=3))
            ids.append(self.canvas.create_line(cx2 - 14, fy, cx2 - 14, fy + prong_len, fill=COLOR_FORK_PRONG, width=3))

        self.canvas.tag_raise("package")
        self.forklift_ids = ids

    # -------------------- phase 1: replay A* expansions --------------------
    def _replay_expansions(self, i):
        if i == 0:
            self._log("=== A* SEARCH STARTED ===", "event")
            self._log(f"Heuristic: Manhattan distance h(n) = |x1-x2| + |y1-y2|")
            self._log(f"Start: {START}   Goal: {GOAL}\n")

        if i >= len(self.trace):
            self._log("\n=== SEARCH COMPLETE ===", "event")
            if self.path:
                self._log(f"Optimal path found — cost {len(self.path) - 1} steps", "goal")
            else:
                self._log("No path found.", "goal")
            self._log(f"Total nodes expanded: {self.expanded_count}")
            self._log(f"Search time: {self.search_time * 1000:.2f} ms\n")
            self._highlight_path()
            self.status_var.set("Delivering package…")
            self.root.after(600, self._animate_move, 0)
            return

        entry = self.trace[i]
        node, g, h, f = entry["node"], entry["g"], entry["h"], entry["f"]

        if node not in (START, GOAL):
            x1, y1 = node[0] * CELL_SIZE, node[1] * CELL_SIZE
            self.canvas.create_rectangle(
                x1 + 2, y1 + 2, x1 + CELL_SIZE - 2, y1 + CELL_SIZE - 2,
                fill=COLOR_VISITED, outline=COLOR_VISITED_EDGE, width=1, tags="visited"
            )
            self.canvas.tag_lower("visited", "package")

        self._log(f"[{entry['order']:>3}] expand {node}  g={g}  h={h}  f={f}")
        self.root.after(EXPANSION_STEP_DELAY, self._replay_expansions, i + 1)

    def _highlight_path(self):
        if not self.path:
            return
        for (x, y) in self.path:
            if (x, y) in (START, GOAL):
                continue
            x1, y1 = x * CELL_SIZE, y * CELL_SIZE
            self.canvas.create_rectangle(
                x1 + 6, y1 + 6, x1 + CELL_SIZE - 6, y1 + CELL_SIZE - 6,
                fill=COLOR_PATH, outline=COLOR_PATH_EDGE, width=1, tags="pathmark"
            )
            self.canvas.tag_lower("pathmark", "package")
        self.cost_var.set(f"Path Cost: {len(self.path) - 1} steps")
        self.nodes_var.set(f"Nodes Expanded: {self.expanded_count}")
        self.time_var.set(f"Time Taken: {self.search_time * 1000:.2f} ms")

    # -------------------- phase 2: drive the forklift --------------------
    def _animate_move(self, step):
        if not self.path or step >= len(self.path):
            self.status_var.set("Delivery complete ✔")
            self._log("=== DELIVERY COMPLETE ===", "event")
            self.replay_btn.config(state="normal")
            return

        cell = self.path[step]

        if step > 0:
            prev = self.path[step - 1]
            self.forklift_dir = (cell[0] - prev[0], cell[1] - prev[1])

        # pick up the package the moment the forklift leaves START
        if cell == START and not self.carrying_package:
            self._log(f"[move {step}] at PICKUP {cell} — package loaded", "move")
        if step == 1 and not self.carrying_package:
            self.carrying_package = True

        self._draw_forklift(cell, self.forklift_dir)
        if self.carrying_package and cell != GOAL:
            self._draw_package(cell, on_forklift=True)
        elif cell == GOAL:
            self._draw_package(cell, on_forklift=False)
            self.carrying_package = False
            self._log(f"[move {step}] arrived at BAY {cell} — package delivered ✔", "move")
        else:
            self._clear_package() if not self.carrying_package else None

        if 0 < step < len(self.path):
            self._log(f"[move {step}] forklift -> {cell}", "move")

        self.root.after(MOVE_STEP_DELAY, self._animate_move, step + 1)

    # -------------------- restart --------------------
    def restart(self):
        self.canvas.delete("visited")
        self.canvas.delete("pathmark")
        self.log_text.delete("1.0", "end")
        self.carrying_package = False
        self.status_var.set("Planning route…")
        self.replay_btn.config(state="disabled")
        self._draw_package(START, on_forklift=False)
        self._draw_forklift(START, (1, 0))
        self.root.after(400, self._replay_expansions, 0)


# ============================================================
# 5. Main Execution
# ============================================================
def main():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for x, y in OBSTACLES:
        grid[y][x] = 1

    t0 = time.perf_counter()
    path, expanded_count, trace = astar(grid, START, GOAL)
    search_time = time.perf_counter() - t0

    # Console output (kept for the required console log / video split-screen)
    if path:
        print(f"Total path cost: {len(path) - 1} steps")
        print(f"Nodes expanded: {expanded_count}")
        print(f"Search time: {search_time * 1000:.2f} ms")
        print(f"Path: {path}")
    else:
        print("No path found.")
        print(f"Nodes expanded: {expanded_count}")

    root = tk.Tk()
    root.title("Warehouse Forklift Simulation — A* Search Agent")
    root.resizable(False, False)

    window_w = GRID_PIXELS + PANEL_WIDTH + 28
    window_h = CANVAS_HEIGHT + 56 + 28
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    x = (screen_w - window_w) // 2
    y = (screen_h - window_h) // 2
    root.geometry(f"{window_w}x{window_h}+{x}+{y}")

    ForkliftSim(root, grid, path, expanded_count, trace, search_time)
    root.mainloop()


if __name__ == "__main__":
    main()