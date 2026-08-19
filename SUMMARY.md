# Technical Summary Sheet — Warehouse Logistics Agent (Track 1)

**Course:** Artificial Intelligence (Sem 5) | **Track:** Unit 2 — Informed Search
**Team Members:** Mithra Bharathi, Unat, [Member 3]
**GitHub:** [https://github.com/mithrabharathi/Hackathon_AI](https://github.com/mithrabharathi/Hackathon_AI)

---

## 1. PEAS Framework

| Component | Description |
|---|---|
| **Performance Measure** | Optimal (shortest) path cost from Start to Loading Bay, minimizing total steps and nodes expanded |
| **Environment** | 10×10 grid warehouse; static shelf obstacles (10 cells); fully observable, deterministic, discrete, single-agent |
| **Actuators** | Forklift movement — 4 directional actions: Up, Down, Left, Right (1 cell per step) |
| **Sensors** | Full grid perception — reads obstacle map, current (x, y) position, and goal coordinates |

---

## 2. Core Algorithmic Formulation

| Element | Definition |
|---|---|
| **State Space** | All 90 free cells in the 10×10 grid (100 total − 10 obstacles). Each state is an (x, y) coordinate |
| **Initial State** | (0, 0) — top-left corner of the warehouse |
| **Goal Test** | Current position = (9, 9) — the designated loading bay |
| **Path Cost** | g(n) = number of moves from start to node n (uniform cost = 1 per step) |
| **Heuristic** | Manhattan Distance: **h(n) = \|x₁ − x₂\| + \|y₁ − y₂\|** |
| **Evaluation Function** | **f(n) = g(n) + h(n)** — estimated total cost through node n |

**Why Manhattan Distance?** It is **admissible** (never overestimates, since diagonal moves are not allowed) and **consistent** (satisfies the triangle inequality), guaranteeing A* finds the optimal path.

---

## 3. Complexity Analysis

**Theoretical (A\* Search — General Case):**
- **Time Complexity:** O(b^d) worst-case, but with a good heuristic effectively O(V log V) where V = reachable cells
- **Space Complexity:** O(V) — stores open set, closed set, g-scores, and came-from map

**Comparison Table — Theoretical vs Observed:**

| Metric | Theoretical (Worst Case) | Observed (10×10 Grid) |
|---|---|---|
| **Time Complexity** | O(b^d) ≈ O(4¹⁸) ≈ 6.87 × 10¹⁰ | **89 node expansions** |
| **Space Complexity** | O(V) = O(90 free cells) | **Max open set: 10 nodes** |
| **Path Cost** | Optimal ≥ Manhattan(start, goal) = 18 | **18 steps (optimal)** |
| **Execution Time** | Dependent on grid scale | **0.25 ms** |
| **Branching Factor (b)** | Up to 4 (cardinal moves) | Effective b ≈ 1.8 (due to obstacles & heuristic pruning) |
| **Solution Depth (d)** | Variable | **18** |
| **Optimality** | Guaranteed (h is admissible) | **Confirmed — cost = h(start) = 18** |

> The heuristic reduces explored nodes from 90 (all free cells) to just 89, confirming near-complete expansion is needed due to obstacle placement. The optimal path cost equals the Manhattan distance of the start-goal pair, verifying zero detour overhead on this configuration.
