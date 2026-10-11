"""
Warehouse Delivery Robot - Search Algorithms
Intelligent Systems - Task 1

Algorithms implemented (no path-finding libraries used):
  1) Uniform Cost Search (UCS)  -> uninformed
  2) A* Search (Manhattan)      -> informed

HOW TO RUN:
    python warehouse_robot.py            # runs the map chosen by GRID below
    python warehouse_robot.py --all      # runs MAP_A, MAP_B, MAP_C and prints a results table

HOW TO CHANGE THE MAP:
    Edit the line  GRID = MAP_A  below and replace MAP_A by MAP_B or MAP_C
    (or define your own list of strings).
"""

import heapq
import sys
import time

# ----------------------------------------------------------------------
# Maps  (S = start, G = goal, . = cost 1, ~ = cost 3, # = wall)
# ----------------------------------------------------------------------
MAP_A = [
    "S..#....",
    "##.#.##.",
    "....~~..",
    ".####.#.",
    "...~..#G",
]

MAP_B = [
    "S~~~~~~~G",
    ".#######.",
    ".........",
    ".#.#.#.#.",
    ".........",
]

MAP_C = [
    "S..#....",
    ".#.#.##.",
    "...#..#.",
    ".###.##.",
    "....#..G",
]

# >>>>>>>>>>  CHANGE THE MAP HERE  <<<<<<<<<<
GRID = MAP_A

MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
COST = {".": 1, "~": 3, "G": 1}              # cost of ENTERING a cell; S is never entered


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
def find_cell(grid, ch):
    """Scan the grid and return (row, col) of the first cell equal to ch."""
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == ch:
                return (r, c)
    raise ValueError(f"Symbol {ch!r} not found in grid")


def neighbors(grid, pos):
    """Yield (next_pos, step_cost) for every legal move from pos."""
    r, c = pos
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[nr]):
            cell = grid[nr][nc]
            if cell != "#":
                yield (nr, nc), COST.get(cell, 1)


def manhattan(a, b):
    """Heuristic: Manhattan distance. Admissible & consistent because the
    cheapest possible move costs 1 and the robot moves in 4 directions."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def rebuild_path(parent, goal):
    """Follow parent pointers back from goal to start."""
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    return path[::-1]


def path_cost(grid, path):
    """Cost of a path = sum of the cost of every cell entered (S not counted)."""
    return sum(COST.get(grid[r][c], 1) for r, c in path[1:])


# ----------------------------------------------------------------------
# Search algorithms
# ----------------------------------------------------------------------
def best_first(grid, informed):
    """
    Generic best-first search.
      informed = False -> UCS : priority f(n) = g(n)
      informed = True  -> A*  : priority f(n) = g(n) + h(n)
    Returns a dict with path, cost, expanded, max_frontier, time.
    """
    start, goal = find_cell(grid, "S"), find_cell(grid, "G")
    t0 = time.perf_counter()

    h = (lambda p: manhattan(p, goal)) if informed else (lambda p: 0)

    counter = 0                                   # tie-breaker for the heap
    frontier = [(h(start), counter, 0, start)]    # (f, tie, g, position)
    parent = {start: None}
    best_g = {start: 0}
    closed = set()
    expanded = 0
    max_frontier = 1

    while frontier:
        f, _, g, pos = heapq.heappop(frontier)
        if pos in closed:                         # stale entry, skip
            continue
        closed.add(pos)
        expanded += 1

        if pos == goal:                           # goal test on expansion -> optimal
            path = rebuild_path(parent, goal)
            return dict(path=path, cost=g, expanded=expanded,
                        max_frontier=max_frontier,
                        time=time.perf_counter() - t0)

        for nxt, step in neighbors(grid, pos):
            ng = g + step
            if nxt not in best_g or ng < best_g[nxt]:
                best_g[nxt] = ng
                parent[nxt] = pos
                counter += 1
                heapq.heappush(frontier, (ng + h(nxt), counter, ng, nxt))
        max_frontier = max(max_frontier, len(frontier))

    return dict(path=None, cost=None, expanded=expanded,
                max_frontier=max_frontier, time=time.perf_counter() - t0)


def ucs(grid):
    return best_first(grid, informed=False)


def astar(grid):
    return best_first(grid, informed=True)


# ----------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------
def show_path(grid, path):
    """Print the map with the path cells marked by *."""
    cells = set(path[1:-1])
    for r, row in enumerate(grid):
        print("   " + "".join("*" if (r, c) in cells else ch
                              for c, ch in enumerate(row)))


def report(name, grid, res):
    print(f"\n=== {name} ===")
    if res["path"] is None:
        print("No path found")
        print(f"Nodes expanded: {res['expanded']}")
        return
    print("Path:", res["path"])
    print("Path length (moves):", len(res["path"]) - 1)
    print("Total cost:", res["cost"])
    print("Nodes expanded:", res["expanded"])
    print("Max frontier size:", res["max_frontier"])
    print(f"Run time: {res['time'] * 1000:.3f} ms")
    show_path(grid, res["path"])


def run_one(grid, label=""):
    print("\n" + "#" * 60)
    print(f"MAP {label}")
    for row in grid:
        print("   " + row)
    results = {}
    for name, fn in (("Uniform Cost Search (UCS)", ucs),
                     ("A* Search (Manhattan)", astar)):
        res = fn(grid)
        report(name, grid, res)
        results[name] = res
    return results


def run_all():
    rows = []
    for label, grid in (("MAP_A", MAP_A), ("MAP_B", MAP_B), ("MAP_C", MAP_C)):
        results = run_one(grid, label)
        for algo, res in results.items():
            rows.append((label, algo, res))

    print("\n\n" + "=" * 92)
    print(f"{'Map':6}{'Algorithm':28}{'Length':>8}{'Cost':>7}{'Expanded':>10}"
          f"{'MaxFrontier':>13}{'Time(ms)':>10}")
    print("-" * 92)
    for label, algo, r in rows:
        length = len(r["path"]) - 1 if r["path"] else "-"
        cost = r["cost"] if r["path"] else "-"
        print(f"{label:6}{algo:28}{str(length):>8}{str(cost):>7}{r['expanded']:>10}"
              f"{r['max_frontier']:>13}{r['time'] * 1000:>10.3f}")
    return rows


if __name__ == "__main__":
    if "--all" in sys.argv:
        run_all()
    else:
        run_one(GRID, "(GRID)")
