#!/usr/bin/env python3
"""
main.py - Run a search algorithm on a map file and print the execution trace.

Usage:
    python3 main.py MAP_FILE [-a {bfs,dfs,astar,all}] [-H {chebyshev,zero}] [-v]

Examples:
    python3 main.py exampleMap.txt -a bfs
    python3 main.py exampleMap.txt -a astar -H chebyshev
    python3 main.py exampleMap.txt -a all
"""
import argparse

from search import ALGORITHMS, HEURISTICS, astar, print_trace, read_map


def main():
    p = argparse.ArgumentParser(description="AIF Lab 1 - drilling robot search")
    p.add_argument("map_file")
    p.add_argument("-a", "--algorithm", default="all",
                   choices=list(ALGORITHMS) + ["all"])
    p.add_argument("-H", "--heuristic", default="chebyshev", choices=list(HEURISTICS))
    p.add_argument("-v", "--verbose", action="store_true",
                   help="print every expansion (debug trace)")
    args = p.parse_args()

    problem = read_map(args.map_file)
    algos = ["bfs", "dfs", "astar"] if args.algorithm == "all" else [args.algorithm]
    for name in algos:
        if name == "astar":
            res = astar(problem, args.heuristic, verbose=args.verbose)
        else:
            res = ALGORITHMS[name](problem, verbose=args.verbose)
        print_trace(res)


if __name__ == "__main__":
    main()
