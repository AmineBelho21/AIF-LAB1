# AIF Lab 1 – Implementation of search algorithms

Artificial Intelligence Fundamentals, Master in Artificial Intelligence 2026-2027.
Author: BELHOCINE Amine.

A drilling robot moves over an N×M map of hardness values (1–9). A state is
`(x, y, o)`, where `o` is one of 8 orientations. The operators are Move (the cost is
the hardness of the destination cell), RotateCW and RotateCCW (cost 1 each).
Implemented methods: breadth-first, depth-first (both as graph search) and A* with
`h(n) = Hmin · max(|x − xt|, |y − yt|)`. Running A* with `h = 0` gives uniform-cost
search, which serves as a baseline.

Requires only Python 3, with no external libraries.

## Usage

```bash
./run.sh                              # traces of BFS, DFS and A* on exampleMap.txt
./run.sh MAP_FILE ALGO [HEUR]         # ALGO in {bfs,dfs,astar,all}, HEUR in {chebyshev,zero}
./run.sh experiments                  # random-map comparison tables (Section 3.4.2)
```

Or call the scripts directly: `python3 main.py exampleMap.txt -a astar -H chebyshev [-v]`.

## Files

| File | Contents |
|---|---|
| `search.py` | Problem model, BFS, DFS, A*, heuristics, trace printing |
| `main.py` | Command-line entry point for one map |
| `experiments.py` | 5 random maps per size N ∈ {3,5,7,9} (seed 2026), averaged tables |
| `run.sh` | Execution script |
| `exampleMap.txt` | Example map from the assignment |
| `maps/` | The random maps used in the experiments |
| `results_raw.csv` | Per-map raw results (d, g, #E, #F) |

## Map format

```
N M
<N rows of M hardness values>
x0 y0 o0        # start (the robot always starts facing North)
xt yt ot        # target; ot = 8 means any orientation
```
