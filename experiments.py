#!/usr/bin/env python3
"""
experiments.py - Section 3.4.2: 5 random maps for each size N in {3,5,7,9},
start (0,0,North), target (N-1,N-1, any orientation). Hardness ~ U{1..9}.
Prints one averaged table per size and saves the maps in ./maps/.
"""
import os
import random

from search import Problem, astar, breadth_first, depth_first

SIZES = [3, 5, 7, 9]
N_MAPS = 5
SEED = 2026

METHODS = [
    ("Breadth-first", lambda p: breadth_first(p)),
    ("Depth-first", lambda p: depth_first(p)),
    ("A* (h = Chebyshev x min hardness)", lambda p: astar(p, "chebyshev")),
    ("A* (h = 0, i.e. uniform cost)", lambda p: astar(p, "zero")),
]


def write_map(path, grid, start, goal):
    n = len(grid)
    with open(path, "w") as f:
        f.write("%d %d\n" % (n, len(grid[0])))
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
        f.write("%d %d %d\n" % start)
        f.write("%d %d %d\n" % goal)


def main():
    rng = random.Random(SEED)
    os.makedirs("maps", exist_ok=True)
    raw = []
    for n in SIZES:
        sums = {name: [0, 0, 0, 0] for name, _ in METHODS}
        for k in range(N_MAPS):
            grid = [[rng.randint(1, 9) for _ in range(n)] for _ in range(n)]
            start, goal = (0, 0, 0), (n - 1, n - 1, 8)
            write_map("maps/map_%dx%d_%d.txt" % (n, n, k + 1), grid, start, goal)
            prob = Problem(grid, start, goal)
            for name, fn in METHODS:
                r = fn(prob)
                vals = [r.node.depth, r.node.g, r.n_explored, r.n_frontier]
                raw.append((n, k + 1, name, *vals))
                for i, v in enumerate(vals):
                    sums[name][i] += v

        print("\nTable: performance of search methods, map %dx%d (mean of %d maps)" % (n, n, N_MAPS))
        print("| Method | d | g | #E | #F |")
        print("|---|---|---|---|---|")
        for name, _ in METHODS:
            d, g, e, f = (v / N_MAPS for v in sums[name])
            print("| %s | %.1f | %.1f | %.1f | %.1f |" % (name, d, g, e, f))

    with open("results_raw.csv", "w") as f:
        f.write("N,map,method,d,g,E,F\n")
        for row in raw:
            f.write("%d,%d,%s,%d,%d,%d,%d\n" % row)
    print("\nPer-map results saved to results_raw.csv")


if __name__ == "__main__":
    main()
