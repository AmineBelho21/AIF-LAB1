#!/usr/bin/env bash
# AIF Lab 1 - execution script. Requires only Python 3 (no external libraries).
#   ./run.sh                          -> traces of BFS, DFS and A* on exampleMap.txt
#   ./run.sh MAP_FILE ALGO [HEUR]     -> ALGO in {bfs,dfs,astar,all}, HEUR in {chebyshev,zero}
#   ./run.sh experiments              -> random-map comparison tables (Section 3.4.2)
set -e
cd "$(dirname "$0")"
if [ "$1" = "experiments" ]; then
    python3 experiments.py
else
    python3 main.py "${1:-exampleMap.txt}" -a "${2:-all}" -H "${3:-chebyshev}"
fi
