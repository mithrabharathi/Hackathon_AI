"""
Warehouse Logistics Agent  |  Track 1 (Unit 2: Informed Search)
----------------------------------------------------------------
Retro pixel-game style UI with A* Search and Manhattan Distance.

Features:
  - Click to place Forklift, Package, and Loading Bay on the grid
  - Two-leg A* pathfinding: Forklift -> Package -> Bay
  - Live decision log showing every A* expansion with g, h, f values
  - Pixel art sprites and retro 8-bit game aesthetic
  - Pipeline phase tracker and real-time metrics
"""

import heapq
import itertools
import time
import os
import tkinter as tk

# Try to load PIL for the forklift sprite image
try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ============================================================
# 1. Grid Constants
# ============================================================
GRID_SIZE = 10
OBSTACLES = {
    (2, 1), (2, 2), (2, 3),
    (4, 5), (4, 6),
    (6, 3), (6, 4), (6, 5),
    (7, 7), (8, 7)
}
CELL = 52
PAD = 8
GRID_PX = GRID_SIZE * CELL
PANEL_W = 340
LOG_H = 220

# Animation timing (ms)
EXPAND_DELAY = 30
MOVE_DELAY = 200

# ============================================================
# 2. Retro Pixel Color Palette
# ============================================================
# -- Backgrounds
C_BG          = "#1a1c2c"    # deep navy
C_PANEL       = "#1f2233"    # panel dark
C_GRID_BG     = "#262b44"    # grid background
C_BORDER      = "#333a55"    # grid lines

# -- Cells
C_FLOOR_A     = "#2a2f4a"    # checkerboard dark
C_FLOOR_B     = "#313759"    # checkerboard light
C_SHELF       = "#5d3a1a"    # brown shelf
C_SHELF_DARK  = "#3e2510"    # shelf outline
C_SHELF_SLAT  = "#7a5230"    # shelf horizontal lines
C_VISITED     = "#1a3050"    # expanded node
C_VISITED_OL  = "#2a5080"    # expanded outline
C_PATH        = "#3a6030"    # path fill
C_PATH_OL     = "#5a9040"    # path outline

# -- Markers
C_START       = "#3b8e3f"    # pickup green
C_START_OL    = "#2d6e30"
C_GOAL        = "#c0392b"    # bay red
C_GOAL_OL     = "#922b21"

# -- Sprites
C_FORK_BODY   = "#e8a838"    # forklift yellow-orange
C_FORK_DARK   = "#b07820"    # forklift outline
C_FORK_CAB    = "#384860"    # cab blue-gray
C_FORK_MAST   = "#8898a8"    # mast silver
C_WHEEL       = "#181818"    # wheels

# -- Package
C_PKG         = "#a05828"    # brown crate
C_PKG_OL      = "#683818"    # crate outline
C_PKG_BAND    = "#d8c8a0"    # packing band

# -- UI accents
C_ACCENT      = "#4fc3f7"    # cyan
C_GREEN       = "#6abe30"    # retro green
C_YELLOW      = "#fbf236"    # retro yellow
C_RED         = "#d95763"    # retro red
C_ORANGE      = "#df7126"    # retro orange
C_TEXT        = "#d0d8e8"    # bright text
C_TEXT_DIM    = "#5a6278"    # dim text
C_LOG_BG      = "#0d1018"    # terminal black
C_LOG_TEXT    = "#50d050"    # terminal green
C_LOG_CYAN    = "#50c8e8"    # log highlights
C_LOG_YELLOW  = "#e8d850"    # log goals
C_LOG_ORANGE  = "#e89030"    # log events

# -- Phase pipeline
PHASE_IDLE         = "idle"
PHASE_ROUTE_PKG    = "route_pkg"
PHASE_PICKUP       = "pickup"
PHASE_ROUTE_BAY    = "route_bay"
PHASE_DELIVERED    = "delivered"
PHASE_LABELS = ["IDLE", "TO PACKAGE", "PICKUP", "TO BAY", "DELIVERED"]
PHASE_KEYS   = [PHASE_IDLE, PHASE_ROUTE_PKG, PHASE_PICKUP, PHASE_ROUTE_BAY, PHASE_DELIVERED]


# ============================================================
# 3. A* Search (with decision trace)
# ============================================================
def astar(grid, start, goal):
    """
    A* Search with Manhattan distance heuristic.
    Returns (path, expanded_count, trace_list).
    trace_list: [{node, g, h, f, order}, ...] for the decision log.
    """
    counter = itertools.count()
    open_set = []
    g_score = {start: 0}
    h0 = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heapq.heappush(open_set, (h0, next(counter), start))
    came_from = {}
    closed = set()
    expanded = 0
    trace = []

    while open_set:
        f, _, current = heapq.heappop(open_set)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        g_here = g_score[current]
        h_here = f - g_here
        trace.append({"node": current, "g": g_here, "h": h_here, "f": f, "order": expanded})

        if current == goal:
            path = []
            cur = goal
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path, expanded, trace

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

    return None, expanded, trace


# ============================================================
# 4. Main Application
# ============================================================
class WarehouseApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Warehouse Logistics  |  A* Agent")
        self.root.configure(bg=C_BG)
        self.root.resizable(False, False)

        # Build obstacle grid
        self.grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
        for x, y in OBSTACLES:
            self.grid[y][x] = 1

        # Placement state
        self.forklift_pos = None
        self.package_pos = None
        self.bay_pos = None
        self.place_mode = "forklift"

        # Simulation state
        self.phase = PHASE_IDLE
        self.leg = 1
        self.leg1_path = []
        self.leg1_trace = []
        self.leg1_expanded = 0
        self.leg2_path = []
        self.leg2_trace = []
        self.leg2_expanded = 0
        self.search_step = 0
        self.move_step = 0
        self.carrying = False
        self.current_pos = None
        self.current_dir = (1, 0)
        self.search_time1 = 0
        self.search_time2 = 0

        # Sprite images
        self.forklift_img = None
        self.forklift_img_left = None
        self.forklift_carry_img = None
        self.forklift_carry_img_left = None
        self._load_sprites()

        # Build UI
        self._build_ui()
        self._draw_grid()
        self._update_phase_bar()
        self._update_placement_btns()

    # ---- Sprite loading ----
    def _load_sprites(self):
        if not HAS_PIL:
            return
        script_dir = os.path.dirname(os.path.abspath(__file__))
        sprite_path = os.path.join(script_dir, "forklift.png")
        if os.path.exists(sprite_path):
            try:
                img = Image.open(sprite_path).convert("RGBA")
                # Resize to fit cell
                size = CELL - 8
                img_r = img.resize((size, size), Image.NEAREST)
                self.forklift_img = ImageTk.PhotoImage(img_r)
                # Flipped for left direction
                img_l = img_r.transpose(Image.FLIP_LEFT_RIGHT)
                self.forklift_img_left = ImageTk.PhotoImage(img_l)
                # For carrying, we tint slightly (just reuse same for now)
                self.forklift_carry_img = self.forklift_img
                self.forklift_carry_img_left = self.forklift_img_left
            except Exception:
                pass

    # ---- UI Construction ----
    def _build_ui(self):
        # Window size
        canvas_w = PAD * 2 + GRID_PX
        win_w = canvas_w + PANEL_W + 12
        win_h = max(canvas_w + 50, 750)  # ensure panel content fits
        sx = (self.root.winfo_screenwidth() - win_w) // 2
        sy = (self.root.winfo_screenheight() - win_h) // 2
        self.root.geometry(f"{win_w}x{win_h}+{sx}+{sy}")

        # -- Header bar --
        header = tk.Frame(self.root, bg="#141628", height=42)
        header.pack(side="top", fill="x")
        header.pack_propagate(False)
        tk.Label(
            header, text="WAREHOUSE LOGISTICS  |  A* AGENT",
            font=("Courier", 12, "bold"), bg="#141628", fg=C_ACCENT
        ).pack(side="left", padx=14, pady=8)
        self.status_var = tk.StringVar(value="Select placement mode, then click grid")
        tk.Label(
            header, textvariable=self.status_var,
            font=("Courier", 9), bg="#141628", fg=C_TEXT_DIM
        ).pack(side="right", padx=14)

        # -- Body --
        body = tk.Frame(self.root, bg=C_BG)
        body.pack(side="top", fill="both", expand=True)

        # Left: Canvas
        left = tk.Frame(body, bg=C_BG, padx=6, pady=6)
        left.pack(side="left", fill="both")
        self.canvas = tk.Canvas(
            left, width=canvas_w, height=canvas_w,
            bg=C_GRID_BG, highlightthickness=0
        )
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self._on_click)

        # Right: Panel
        panel = tk.Frame(body, bg=C_PANEL, width=PANEL_W)
        panel.pack(side="right", fill="both", expand=True, padx=(0, 6), pady=6)
        panel.pack_propagate(False)

        # -- Placement Buttons --
        place_frame = tk.Frame(panel, bg=C_PANEL)
        place_frame.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(
            place_frame, text="PLACEMENT", font=("Courier", 8, "bold"),
            bg=C_PANEL, fg=C_TEXT_DIM
        ).pack(anchor="w")

        btn_row = tk.Frame(place_frame, bg=C_PANEL)
        btn_row.pack(fill="x", pady=(4, 0))

        self.btn_forklift = tk.Button(
            btn_row, text="FORKLIFT", font=("Courier", 8, "bold"),
            relief="flat", cursor="hand2", padx=4, pady=3,
            command=lambda: self._set_mode("forklift")
        )
        self.btn_forklift.pack(side="left", expand=True, fill="x", padx=(0, 2))

        self.btn_package = tk.Button(
            btn_row, text="PACKAGE", font=("Courier", 8, "bold"),
            relief="flat", cursor="hand2", padx=4, pady=3,
            command=lambda: self._set_mode("package")
        )
        self.btn_package.pack(side="left", expand=True, fill="x", padx=1)

        self.btn_bay = tk.Button(
            btn_row, text="BAY", font=("Courier", 8, "bold"),
            relief="flat", cursor="hand2", padx=4, pady=3,
            command=lambda: self._set_mode("bay")
        )
        self.btn_bay.pack(side="left", expand=True, fill="x", padx=(2, 0))

        # -- Separator --
        tk.Frame(panel, bg=C_BORDER, height=1).pack(fill="x", padx=10, pady=4)

        # -- Pipeline --
        pipe_frame = tk.Frame(panel, bg=C_PANEL)
        pipe_frame.pack(fill="x", padx=10)
        tk.Label(
            pipe_frame, text="PIPELINE", font=("Courier", 8, "bold"),
            bg=C_PANEL, fg=C_TEXT_DIM
        ).pack(anchor="w")

        self.phase_bar = []
        for lbl in PHASE_LABELS:
            row = tk.Frame(pipe_frame, bg=C_PANEL)
            row.pack(fill="x")
            dot = tk.Label(row, text=">", bg=C_PANEL, fg=C_BORDER, font=("Courier", 9, "bold"))
            dot.pack(side="left")
            ltxt = tk.Label(row, text=lbl, bg=C_PANEL, fg=C_TEXT_DIM, font=("Courier", 8))
            ltxt.pack(side="left", padx=4)
            self.phase_bar.append((dot, ltxt))

        # -- Separator --
        tk.Frame(panel, bg=C_BORDER, height=1).pack(fill="x", padx=10, pady=4)

        # -- Metrics --
        met_frame = tk.Frame(panel, bg=C_PANEL)
        met_frame.pack(fill="x", padx=10)
        tk.Label(
            met_frame, text="METRICS", font=("Courier", 8, "bold"),
            bg=C_PANEL, fg=C_TEXT_DIM
        ).pack(anchor="w")

        self.cost_var = tk.StringVar(value="--")
        self.nodes_var = tk.StringVar(value="--")
        self.time_var = tk.StringVar(value="--")

        for label, var, color in [
            ("PATH COST", self.cost_var, C_GREEN),
            ("NODES EXPANDED", self.nodes_var, C_ACCENT),
            ("SEARCH TIME", self.time_var, C_YELLOW),
        ]:
            f = tk.Frame(met_frame, bg=C_PANEL)
            f.pack(fill="x", pady=1)
            tk.Label(f, text=label, bg=C_PANEL, fg=C_TEXT_DIM, font=("Courier", 7)).pack(anchor="w")
            tk.Label(f, textvariable=var, bg=C_PANEL, fg=color, font=("Courier", 11, "bold")).pack(anchor="w")

        # -- Separator --
        tk.Frame(panel, bg=C_BORDER, height=1).pack(fill="x", padx=10, pady=4)

        # -- Decision Log --
        log_frame_outer = tk.Frame(panel, bg=C_PANEL)
        log_frame_outer.pack(fill="both", expand=True, padx=10, pady=(0, 4))
        tk.Label(
            log_frame_outer, text="DECISION LOG", font=("Courier", 8, "bold"),
            bg=C_PANEL, fg=C_TEXT_DIM
        ).pack(anchor="w")

        log_inner = tk.Frame(log_frame_outer, bg=C_LOG_BG, bd=2, relief="sunken")
        log_inner.pack(fill="both", expand=True, pady=(2, 0))
        scrollbar = tk.Scrollbar(log_inner, troughcolor=C_LOG_BG)
        scrollbar.pack(side="right", fill="y")
        self.log_text = tk.Text(
            log_inner, bg=C_LOG_BG, fg=C_LOG_TEXT,
            font=("Courier", 8), wrap="none", relief="flat",
            insertbackground=C_LOG_TEXT, yscrollcommand=scrollbar.set,
            padx=4, pady=4
        )
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)

        # Log tags
        self.log_text.tag_config("event", foreground=C_LOG_ORANGE)
        self.log_text.tag_config("goal", foreground=C_LOG_YELLOW)
        self.log_text.tag_config("move", foreground=C_LOG_CYAN)
        self.log_text.tag_config("info", foreground=C_TEXT_DIM)

        # -- Separator --
        tk.Frame(panel, bg=C_BORDER, height=1).pack(fill="x", padx=10, pady=2)

        # -- Buttons --
        btn_frame = tk.Frame(panel, bg=C_PANEL)
        btn_frame.pack(fill="x", padx=10, pady=(2, 8))

        self.run_btn = tk.Button(
            btn_frame, text="RUN SIMULATION", font=("Courier", 10, "bold"),
            bg=C_GREEN, fg="#0a0a0a", relief="flat", cursor="hand2",
            activebackground="#8ade50", padx=8, pady=6,
            command=self._run_simulation, state="disabled"
        )
        self.run_btn.pack(fill="x", pady=(0, 4))

        self.reset_btn = tk.Button(
            btn_frame, text="RESET", font=("Courier", 9, "bold"),
            bg=C_BORDER, fg=C_TEXT, relief="flat", cursor="hand2",
            activebackground="#4a5070", padx=8, pady=4,
            command=self._reset
        )
        self.reset_btn.pack(fill="x")

        # Initial log message
        self._log("--- Warehouse Logistics Agent ---", "event")
        self._log("A* Search | Manhattan Distance", "info")
        self._log("")
        self._log("1. Select FORKLIFT, PACKAGE, BAY", "info")
        self._log("2. Click cells on the grid", "info")
        self._log("3. Press RUN SIMULATION", "info")

    # ---- Logging ----
    def _log(self, msg, tag=None):
        self.log_text.insert("end", msg + "\n", tag)
        self.log_text.see("end")

    # ---- Placement ----
    def _set_mode(self, mode):
        if self.phase not in (PHASE_IDLE, PHASE_DELIVERED):
            return
        self.place_mode = mode
        self._update_placement_btns()

    def _update_placement_btns(self):
        for btn, mode, active_bg, active_fg in [
            (self.btn_forklift, "forklift", C_ACCENT, "#0a0a0a"),
            (self.btn_package, "package", C_ORANGE, "#0a0a0a"),
            (self.btn_bay, "bay", C_RED, "#ffffff"),
        ]:
            if self.place_mode == mode:
                btn.config(bg=active_bg, fg=active_fg)
            else:
                btn.config(bg=C_BORDER, fg=C_TEXT_DIM)

    def _on_click(self, event):
        if self.phase not in (PHASE_IDLE, PHASE_DELIVERED):
            return
        x = (event.x - PAD) // CELL
        y = (event.y - PAD) // CELL
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return
        if (x, y) in OBSTACLES:
            self._log("  Blocked cell -- select a free one.", "event")
            return

        # Clear if occupied
        if (x, y) == self.forklift_pos:
            self.forklift_pos = None
        if (x, y) == self.package_pos:
            self.package_pos = None
        if (x, y) == self.bay_pos:
            self.bay_pos = None

        if self.place_mode == "forklift":
            self.forklift_pos = (x, y)
            self._log(f"  Forklift placed at ({x},{y})", "goal")
        elif self.place_mode == "package":
            self.package_pos = (x, y)
            self._log(f"  Package placed at ({x},{y})", "goal")
        elif self.place_mode == "bay":
            self.bay_pos = (x, y)
            self._log(f"  Bay placed at ({x},{y})", "goal")

        # Check readiness
        all_set = all([self.forklift_pos, self.package_pos, self.bay_pos])
        self.run_btn.config(state="normal" if all_set else "disabled")
        self.status_var.set("Ready -- press RUN" if all_set else "Place all 3 markers on the grid")
        self._draw_grid()

    # ---- Phase Bar ----
    def _update_phase_bar(self):
        idx = PHASE_KEYS.index(self.phase) if self.phase in PHASE_KEYS else 0
        for i, (dot, lbl) in enumerate(self.phase_bar):
            if i < idx:
                dot.config(fg=C_GREEN)
                lbl.config(fg=C_TEXT_DIM)
            elif i == idx:
                dot.config(fg=C_ACCENT)
                lbl.config(fg=C_TEXT)
            else:
                dot.config(fg=C_BORDER)
                lbl.config(fg=C_BORDER)

    # ---- Grid Drawing ----
    def _cell_xy(self, x, y):
        x1 = PAD + x * CELL
        y1 = PAD + y * CELL
        return x1, y1, x1 + CELL, y1 + CELL

    def _cell_center(self, x, y):
        x1, y1, x2, y2 = self._cell_xy(x, y)
        return (x1 + x2) // 2, (y1 + y2) // 2

    def _draw_grid(self):
        c = self.canvas
        c.delete("all")
        canvas_size = PAD * 2 + GRID_PX

        # Background
        c.create_rectangle(0, 0, canvas_size, canvas_size, fill=C_GRID_BG, outline="")

        # Determine current path and visited sets
        if self.leg == 1:
            visited_set = set(
                e["node"] for e in self.leg1_trace[:self.search_step]
            ) if self.leg1_trace else set()
            path_set = set(self.leg1_path) if (
                self.phase != PHASE_ROUTE_PKG or self.search_step >= len(self.leg1_trace)
            ) else set()
            path_list = self.leg1_path
        else:
            visited_set = set(
                e["node"] for e in self.leg2_trace[:self.search_step]
            ) if self.leg2_trace else set()
            path_set = set(self.leg2_path) if (
                self.phase != PHASE_ROUTE_BAY or self.search_step >= len(self.leg2_trace)
            ) else set()
            path_list = self.leg2_path

        # Draw cells
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                cell = (x, y)
                x1, y1, x2, y2 = self._cell_xy(x, y)

                if cell in OBSTACLES:
                    self._draw_shelf(x, y)
                elif cell in path_set and cell not in (self.forklift_pos, self.package_pos, self.bay_pos):
                    c.create_rectangle(x1, y1, x2, y2, fill=C_PATH, outline=C_PATH_OL, width=1)
                    # Direction arrow
                    if cell in path_set and path_list:
                        try:
                            idx = path_list.index(cell)
                            if idx < len(path_list) - 1:
                                nx, ny = path_list[idx + 1]
                                dx, dy = nx - x, ny - y
                                arrows = {(1,0): ">", (-1,0): "<", (0,1): "v", (0,-1): "^"}
                                arr = arrows.get((dx, dy), "")
                                cx, cy = self._cell_center(x, y)
                                c.create_text(cx, cy, text=arr, fill=C_GREEN, font=("Courier", 14, "bold"))
                        except ValueError:
                            pass
                elif cell in visited_set and cell not in (self.forklift_pos, self.package_pos, self.bay_pos):
                    c.create_rectangle(x1, y1, x2, y2, fill=C_VISITED, outline=C_VISITED_OL, width=1)
                    cx, cy = self._cell_center(x, y)
                    c.create_text(cx, cy, text="+", fill=C_VISITED_OL, font=("Courier", 10))
                else:
                    fill = C_FLOOR_A if (x + y) % 2 == 0 else C_FLOOR_B
                    c.create_rectangle(x1, y1, x2, y2, fill=fill, outline=C_BORDER, width=1)
                    # Coordinate labels in idle
                    if self.phase in (PHASE_IDLE, PHASE_DELIVERED):
                        cx, cy = self._cell_center(x, y)
                        c.create_text(cx, cy, text=f"{x},{y}", fill="#303858", font=("Courier", 7))

        # Draw markers
        if self.forklift_pos and self.current_pos is None:
            self._draw_start_marker(*self.forklift_pos)
        if self.package_pos and not self.carrying:
            self._draw_package_marker(*self.package_pos)
        if self.bay_pos:
            self._draw_bay_marker(*self.bay_pos)

        # Draw delivered package at bay
        if self.phase == PHASE_DELIVERED and self.bay_pos:
            self._draw_delivered_pkg(*self.bay_pos)

        # Draw forklift sprite if active
        if self.current_pos is not None:
            self._draw_forklift_sprite(*self.current_pos)

        # Grid border (retro double-border)
        c.create_rectangle(PAD - 2, PAD - 2, PAD + GRID_PX + 1, PAD + GRID_PX + 1,
                           outline=C_ACCENT, width=2)
        c.create_rectangle(PAD - 4, PAD - 4, PAD + GRID_PX + 3, PAD + GRID_PX + 3,
                           outline=C_BORDER, width=1)

    def _draw_shelf(self, x, y):
        c = self.canvas
        x1, y1, x2, y2 = self._cell_xy(x, y)
        # Base
        c.create_rectangle(x1 + 2, y1 + 2, x2 - 2, y2 - 2,
                           fill=C_SHELF, outline=C_SHELF_DARK, width=2)
        # Slats
        for i in range(1, 4):
            sy = y1 + i * (CELL // 4)
            c.create_line(x1 + 5, sy, x2 - 5, sy, fill=C_SHELF_SLAT, width=2)
        # Tiny boxes on shelves
        c.create_rectangle(x1 + 8, y1 + 6, x1 + 18, y1 + 12, fill="#c0a060", outline=C_SHELF_DARK)
        c.create_rectangle(x1 + 22, y1 + 6, x1 + 34, y1 + 12, fill="#a08848", outline=C_SHELF_DARK)
        c.create_rectangle(x1 + 10, y1 + CELL // 4 + 4, x1 + 24, y1 + CELL // 4 + 10,
                           fill="#b89858", outline=C_SHELF_DARK)

    def _draw_start_marker(self, x, y):
        c = self.canvas
        x1, y1, x2, y2 = self._cell_xy(x, y)
        c.create_rectangle(x1 + 3, y1 + 3, x2 - 3, y2 - 3,
                           fill=C_START, outline=C_START_OL, width=2)
        cx, cy = self._cell_center(x, y)
        c.create_text(cx, cy - 4, text="START", fill="#ffffff", font=("Courier", 8, "bold"))
        c.create_text(cx, cy + 8, text=">>", fill="#80e080", font=("Courier", 9, "bold"))

    def _draw_package_marker(self, x, y):
        c = self.canvas
        cx, cy = self._cell_center(x, y)
        sz = 18
        # Crate
        c.create_rectangle(cx - sz, cy - sz, cx + sz, cy + sz,
                           fill=C_PKG, outline=C_PKG_OL, width=2)
        # Cross bands
        c.create_line(cx - sz, cy, cx + sz, cy, fill=C_PKG_BAND, width=2)
        c.create_line(cx, cy - sz, cx, cy + sz, fill=C_PKG_BAND, width=2)
        # Label
        c.create_text(cx, cy + sz + 8, text="PKG", fill=C_ORANGE, font=("Courier", 7, "bold"))

    def _draw_bay_marker(self, x, y):
        c = self.canvas
        x1, y1, x2, y2 = self._cell_xy(x, y)
        c.create_rectangle(x1 + 3, y1 + 3, x2 - 3, y2 - 3,
                           fill=C_GOAL, outline=C_GOAL_OL, width=2)
        # Hazard stripes
        for i in range(-2, 6):
            sx = x1 + 5 + i * 12
            c.create_line(sx, y1 + 4, sx + 10, y2 - 4, fill="#e06050", width=3)
        # Redraw border on top
        c.create_rectangle(x1 + 3, y1 + 3, x2 - 3, y2 - 3,
                           fill="", outline=C_GOAL_OL, width=2)
        cx, cy = self._cell_center(x, y)
        c.create_text(cx, cy, text="BAY", fill="#ffffff", font=("Courier", 9, "bold"))

    def _draw_delivered_pkg(self, x, y):
        c = self.canvas
        cx, cy = self._cell_center(x, y)
        c.create_rectangle(cx - 8, cy + 6, cx + 8, cy + 18,
                           fill=C_PKG, outline=C_PKG_OL, width=1)
        c.create_line(cx - 8, cy + 12, cx + 8, cy + 12, fill=C_PKG_BAND, width=1)

    def _draw_forklift_sprite(self, x, y):
        c = self.canvas
        cx, cy = self._cell_center(x, y)
        dx, _ = self.current_dir

        # Try using loaded sprite image
        if self.forklift_img is not None:
            if self.carrying:
                img = self.forklift_carry_img_left if dx < 0 else self.forklift_carry_img
            else:
                img = self.forklift_img_left if dx < 0 else self.forklift_img
            if img:
                c.create_image(cx, cy, image=img, anchor="center")
                # Draw small package on top if carrying
                if self.carrying:
                    c.create_rectangle(cx - 6, cy - 20, cx + 6, cy - 12,
                                       fill=C_PKG, outline=C_PKG_OL, width=1)
                    c.create_line(cx - 6, cy - 16, cx + 6, cy - 16, fill=C_PKG_BAND, width=1)
                return

        # Fallback: draw pixel forklift with canvas primitives
        self._draw_forklift_canvas(x, y)

    def _draw_forklift_canvas(self, x, y):
        """Pixel art forklift drawn with canvas rectangles."""
        c = self.canvas
        cx, cy = self._cell_center(x, y)
        dx, dy = self.current_dir

        # Wheels
        if dx != 0:  # horizontal
            c.create_oval(cx - 12, cy + 8, cx - 4, cy + 16, fill=C_WHEEL, outline="#080808")
            c.create_oval(cx + 4, cy + 8, cx + 12, cy + 16, fill=C_WHEEL, outline="#080808")
        else:  # vertical
            c.create_oval(cx - 16, cy - 6, cx - 8, cy + 2, fill=C_WHEEL, outline="#080808")
            c.create_oval(cx + 8, cy - 6, cx + 16, cy + 2, fill=C_WHEEL, outline="#080808")

        # Body
        c.create_rectangle(cx - 14, cy - 8, cx + 8, cy + 10,
                           fill=C_FORK_BODY, outline=C_FORK_DARK, width=2)
        # Cab
        c.create_rectangle(cx - 12, cy - 14, cx - 2, cy - 4,
                           fill=C_FORK_CAB, outline="#2a3848", width=1)
        # Mast
        c.create_line(cx + 8, cy - 12, cx + 8, cy + 8, fill=C_FORK_MAST, width=3)
        # Forks
        fork_y = cy + 4
        c.create_line(cx + 8, fork_y - 3, cx + 20, fork_y - 3, fill=C_FORK_MAST, width=2)
        c.create_line(cx + 8, fork_y + 3, cx + 20, fork_y + 3, fill=C_FORK_MAST, width=2)
        # Headlight
        c.create_rectangle(cx + 5, cy - 6, cx + 8, cy - 3, fill=C_YELLOW, outline="")

        # Package on forks
        if self.carrying:
            c.create_rectangle(cx + 10, fork_y - 8, cx + 22, fork_y + 6,
                               fill=C_PKG, outline=C_PKG_OL, width=1)
            c.create_line(cx + 10, fork_y - 1, cx + 22, fork_y - 1, fill=C_PKG_BAND, width=1)

    # ---- Simulation ----
    def _run_simulation(self):
        if not all([self.forklift_pos, self.package_pos, self.bay_pos]):
            return

        # Reset state
        self.phase = PHASE_ROUTE_PKG
        self.leg = 1
        self.search_step = 0
        self.move_step = 0
        self.carrying = False
        self.current_pos = self.forklift_pos
        self.current_dir = (1, 0)

        self.run_btn.config(state="disabled", text="RUNNING...")
        self.btn_forklift.config(state="disabled")
        self.btn_package.config(state="disabled")
        self.btn_bay.config(state="disabled")

        # Clear log
        self.log_text.delete("1.0", "end")
        self.cost_var.set("--")
        self.nodes_var.set("--")
        self.time_var.set("--")

        # Run A* for both legs
        t0 = time.perf_counter()
        self.leg1_path, self.leg1_expanded, self.leg1_trace = astar(
            self.grid, self.forklift_pos, self.package_pos
        )
        self.search_time1 = time.perf_counter() - t0

        t0 = time.perf_counter()
        self.leg2_path, self.leg2_expanded, self.leg2_trace = astar(
            self.grid, self.package_pos, self.bay_pos
        )
        self.search_time2 = time.perf_counter() - t0

        if not self.leg1_path:
            self._log("No path to package.", "event")
            self.status_var.set("No path to package")
            self._enable_ui()
            return
        if not self.leg2_path:
            self._log("No path to bay.", "event")
            self.status_var.set("No path to bay")
            self._enable_ui()
            return

        # Console output
        total_cost = (len(self.leg1_path) - 1) + (len(self.leg2_path) - 1)
        total_expanded = self.leg1_expanded + self.leg2_expanded
        total_time = self.search_time1 + self.search_time2
        print(f"Leg 1: cost={len(self.leg1_path)-1}, expanded={self.leg1_expanded}")
        print(f"Leg 2: cost={len(self.leg2_path)-1}, expanded={self.leg2_expanded}")
        print(f"Total: cost={total_cost}, expanded={total_expanded}, time={total_time*1000:.2f}ms")

        self.cost_var.set(f"{total_cost} steps")
        self.nodes_var.set(f"{total_expanded}")
        self.time_var.set(f"{total_time * 1000:.2f} ms")

        self.status_var.set("Searching path to package...")
        self._update_phase_bar()
        self._draw_grid()

        # Start leg 1 search replay
        self.root.after(300, self._tick_search)

    def _tick_search(self):
        trace = self.leg1_trace if self.leg == 1 else self.leg2_trace

        if self.search_step == 0:
            leg_label = "package" if self.leg == 1 else "bay"
            start = self.forklift_pos if self.leg == 1 else self.package_pos
            goal = self.package_pos if self.leg == 1 else self.bay_pos
            self._log(f"--- A* Leg {self.leg}: to {leg_label} ---", "event")
            self._log(f"h(n) = |x1-x2| + |y1-y2|")
            self._log(f"From {start}  to  {goal}")
            self._log("")

        if self.search_step < len(trace):
            entry = trace[self.search_step]
            self._log(
                f"[{entry['order']:>3}] ({entry['node'][0]},{entry['node'][1]})"
                f"  g={entry['g']} h={entry['h']} f={entry['f']}"
            )
            self.search_step += 1
            self.nodes_var.set(f"{self.search_step}")
            self._draw_grid()
            delay = max(10, EXPAND_DELAY)
            self.root.after(delay, self._tick_search)
        else:
            # Search complete for this leg
            path = self.leg1_path if self.leg == 1 else self.leg2_path
            cost = len(path) - 1
            self._log("")
            self._log(f"--- Search complete ---", "event")
            self._log(f"Path found, cost: {cost} steps", "goal")
            self._log("")
            self._draw_grid()
            self.root.after(400, self._start_move)

    def _start_move(self):
        self.move_step = 0
        self._tick_move()

    def _tick_move(self):
        path = self.leg1_path if self.leg == 1 else self.leg2_path

        if self.move_step >= len(path):
            if self.leg == 1:
                # Arrived at package -- pickup
                self._do_pickup()
            else:
                # Arrived at bay -- delivered
                self._do_delivered()
            return

        cell = path[self.move_step]

        # Update direction
        if self.move_step > 0:
            prev = path[self.move_step - 1]
            self.current_dir = (cell[0] - prev[0], cell[1] - prev[1])

        self.current_pos = cell
        label = "to package" if self.leg == 1 else "to bay"
        self._log(f"[move] -> ({cell[0]},{cell[1]})  {label}", "move")
        self.status_var.set(f"Moving {label}... step {self.move_step + 1}/{len(path)}")
        self._draw_grid()
        self.move_step += 1
        self.root.after(MOVE_DELAY, self._tick_move)

    def _do_pickup(self):
        self.phase = PHASE_PICKUP
        self.carrying = True
        self._update_phase_bar()
        self._log("")
        self._log("--- Package loaded ---", "event")
        self._log("")
        self.status_var.set("Package picked up -- routing to bay...")
        self._draw_grid()

        # Transition to leg 2
        self.root.after(500, self._start_leg2)

    def _start_leg2(self):
        self.phase = PHASE_ROUTE_BAY
        self.leg = 2
        self.search_step = 0
        self._update_phase_bar()
        self.status_var.set("Searching path to bay...")
        self._draw_grid()
        self.root.after(200, self._tick_search)

    def _do_delivered(self):
        self.phase = PHASE_DELIVERED
        self.carrying = False
        self._update_phase_bar()
        self._log("")
        self._log("--- Package delivered ---", "event")
        total_cost = (len(self.leg1_path) - 1) + (len(self.leg2_path) - 1)
        total_exp = self.leg1_expanded + self.leg2_expanded
        total_time = self.search_time1 + self.search_time2
        self._log(f"Total cost:     {total_cost} steps", "goal")
        self._log(f"Nodes expanded: {total_exp}", "goal")
        self._log(f"Search time:    {total_time * 1000:.2f} ms", "goal")
        self.status_var.set("Delivery complete")
        self._draw_grid()
        self._enable_ui()

    def _enable_ui(self):
        self.run_btn.config(state="normal", text="RUN AGAIN")
        self.btn_forklift.config(state="normal")
        self.btn_package.config(state="normal")
        self.btn_bay.config(state="normal")
        self._update_placement_btns()

    def _reset(self):
        self.forklift_pos = None
        self.package_pos = None
        self.bay_pos = None
        self.phase = PHASE_IDLE
        self.leg = 1
        self.search_step = 0
        self.move_step = 0
        self.carrying = False
        self.current_pos = None
        self.current_dir = (1, 0)
        self.leg1_path = []
        self.leg1_trace = []
        self.leg1_expanded = 0
        self.leg2_path = []
        self.leg2_trace = []
        self.leg2_expanded = 0

        self.cost_var.set("--")
        self.nodes_var.set("--")
        self.time_var.set("--")
        self.status_var.set("Select placement mode, then click grid")
        self.run_btn.config(state="disabled", text="RUN SIMULATION")

        self.log_text.delete("1.0", "end")
        self._log("--- Reset ---", "event")
        self._log("Place FORKLIFT, PACKAGE, BAY", "info")
        self._log("Then press RUN SIMULATION", "info")

        self._enable_ui()
        self._draw_grid()
        self._update_phase_bar()


# ============================================================
# 5. Main
# ============================================================
def main():
    root = tk.Tk()
    WarehouseApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()