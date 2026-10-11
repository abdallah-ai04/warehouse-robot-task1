# Warehouse Delivery Robot - Search Algorithms

Intelligent Systems - Task 1 (Alexandria University)

A robot moves from S to G on a grid (Up/Down/Left/Right). Normal floor `.` costs 1, rough floor `~` costs 3, wall `#` is blocked.

## Algorithms
- **UCS** (Uniform Cost Search) - uninformed
- **A\*** with Manhattan distance heuristic - informed

No path-finding libraries are used (only `heapq` and `time` from the standard library).

## How to run
```bash
python warehouse_robot.py          # runs the map selected by GRID
python warehouse_robot.py --all    # runs MAP_A, MAP_B, MAP_C and prints the results table
```

## How to change the map
Open `warehouse_robot.py` and edit this line near the top:
```python
GRID = MAP_A   # change to MAP_B, MAP_C, or your own list of strings
```

## Files
- `warehouse_robot.py` - source code
- `Report_Warehouse_Robot.pdf` - report (formulation, results, comparison, conclusion)
