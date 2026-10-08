"""
search.py - Core search infrastructure for AIF Lab 1 (drilling robot).

State  S = (x, y, o)
    x : row index    (0 = top row)
    y : column index (0 = left-most column)
    o : orientation  0..7 = N, NE, E, SE, S, SW, W, NW

Operators (expansion order): Move, RotateCW, RotateCCW
    Move      : drill one cell forward along o; cost = hardness of the destination cell
    RotateCW  : o <- (o + 1) mod 8 ; cost = 1
    RotateCCW : o <- (o - 1) mod 8 ; cost = 1

Goal test: (x, y) == (xt, yt) and (ot == 8 or o == ot)
"""
import heapq
import itertools
from collections import deque

ORIENTATIONS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
# (delta_row, delta_col) for each orientation; row 0 is the top of the map
DELTAS = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
ANY_ORIENTATION = 8


# --------------------------------------------------------------------------- #
# Problem definition
# --------------------------------------------------------------------------- #
class Problem:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.min_hardness = min(min(row) for row in grid)

    def actions(self, state):
        """Applicable operators. Move requires the next cell to be inside the map."""
        x, y, o = state
        dx, dy = DELTAS[o]
        acts = []
        if 0 <= x + dx < self.rows and 0 <= y + dy < self.cols:
            acts.append("Move")
        acts.extend(("RotateCW", "RotateCCW"))
        return acts

    def result(self, state, action):
        """Transition model."""
        x, y, o = state
        if action == "Move":
            dx, dy = DELTAS[o]
            return (x + dx, y + dy, o)
        if action == "RotateCW":
            return (x, y, (o + 1) % 8)
        if action == "RotateCCW":
            return (x, y, (o - 1) % 8)
        raise ValueError("Unknown action: %s" % action)

    def step_cost(self, state, action, new_state):
        if action == "Move":
            return self.grid[new_state[0]][new_state[1]]
        return 1

    def goal_test(self, state):
        gx, gy, go = self.goal
        return (state[0] == gx and state[1] == gy
                and (go == ANY_ORIENTATION or state[2] == go))


# --------------------------------------------------------------------------- #
# Search-tree node  (n.STATE, n.PARENT, n.ACTION, n.PATH-COST, depth, h)
# --------------------------------------------------------------------------- #
class Node:
    __slots__ = ("state", "parent", "action", "g", "depth", "h")

    def __init__(self, state, parent=None, action=None, g=0, depth=0, h=None):
        self.state = state
        self.parent = parent
        self.action = action
        self.g = g
        self.depth = depth
        self.h = h

    def path(self):
        node, out = self, []
        while node is not None:
            out.append(node)
            node = node.parent
        return list(reversed(out))


def child_node(problem, parent, action, h=None):
    s2 = problem.result(parent.state, action)
    g = parent.g + problem.step_cost(parent.state, action, s2)
    return Node(s2, parent, action, g, parent.depth + 1,
                None if h is None else h(s2))


class SearchResult:
    def __init__(self, algorithm, solved, node, n_explored, n_frontier, informed=False):
        self.algorithm = algorithm
        self.solved = solved          # True if a goal was reached
        self.node = node              # goal node, or last examined node on failure
        self.n_explored = n_explored  # |explored| at the end
        self.n_frontier = n_frontier  # |frontier| at the end
        self.informed = informed


# --------------------------------------------------------------------------- #
# Heuristics
# --------------------------------------------------------------------------- #
def h_chebyshev(problem):
    """h(n) = min_hardness * max(|x - xt|, |y - yt|)  (admissible and consistent)."""
    gx, gy, _ = problem.goal
    m = problem.min_hardness
    return lambda s: m * max(abs(s[0] - gx), abs(s[1] - gy))


def h_zero(problem):
    """h(n) = 0 -> A* degenerates into uniform-cost search (baseline)."""
    return lambda s: 0


HEURISTICS = {"chebyshev": h_chebyshev, "zero": h_zero}


# --------------------------------------------------------------------------- #
# Blind search: breadth-first and depth-first (graph search, AIMA style)
# Goal test is applied when a node is generated.
# --------------------------------------------------------------------------- #
def _blind_search(problem, lifo, verbose=False):
    name = "Depth-first" if lifo else "Breadth-first"
    root = Node(problem.start)
    if problem.goal_test(root.state):
        return SearchResult(name, True, root, 0, 0)

    frontier = deque([root])
    in_frontier = {root.state}
    explored = set()
    last = root
    it = 0
    while frontier:
        node = frontier.pop() if lifo else frontier.popleft()
        in_frontier.discard(node.state)
        explored.add(node.state)
        last = node
        it += 1
        if verbose:
            print("[it %d] expand %s  |explored|=%d |frontier|=%d"
                  % (it, format_node(node, False), len(explored), len(frontier)))

        children = []
        for action in problem.actions(node.state):
            child = child_node(problem, node, action)
            if child.state in explored or child.state in in_frontier:
                continue
            if problem.goal_test(child.state):
                return SearchResult(name, True, child, len(explored), len(frontier))
            children.append(child)

        # For DFS push in reverse so that the first operator (Move) is popped first.
        for child in (reversed(children) if lifo else children):
            frontier.append(child)
            in_frontier.add(child.state)

    return SearchResult(name, False, last, len(explored), 0)


def breadth_first(problem, verbose=False):
    return _blind_search(problem, lifo=False, verbose=verbose)


def depth_first(problem, verbose=False):
    return _blind_search(problem, lifo=True, verbose=verbose)


# --------------------------------------------------------------------------- #
# A*  (best-first with f = g + h). Goal test when a node is selected for expansion.
# Ties on f are broken by lower h, then FIFO.
# --------------------------------------------------------------------------- #
def astar(problem, heuristic="chebyshev", verbose=False):
    h = HEURISTICS[heuristic](problem)
    counter = itertools.count()
    root = Node(problem.start, h=h(problem.start))
    frontier = [(root.g + root.h, root.h, next(counter), root)]
    best = {root.state: root}      # best node currently in the frontier per state
    explored = {}                  # state -> g with which it was expanded
    last = root
    it = 0
    while frontier:
        _, _, _, node = heapq.heappop(frontier)
        if best.get(node.state) is not node:
            continue               # stale heap entry (a better one was pushed)
        del best[node.state]
        last = node
        if problem.goal_test(node.state):
            return SearchResult("A* (%s)" % heuristic, True, node,
                                len(explored), len(best), informed=True)
        explored[node.state] = node.g
        it += 1
        if verbose:
            print("[it %d] expand %s  |explored|=%d |frontier|=%d"
                  % (it, format_node(node, True), len(explored), len(best)))

        for action in problem.actions(node.state):
            child = child_node(problem, node, action, h)
            s = child.state
            if s in explored:
                if child.g < explored[s]:
                    del explored[s]    # re-open (never happens with a consistent h)
                else:
                    continue
            if s in best and best[s].g <= child.g:
                continue
            best[s] = child
            heapq.heappush(frontier, (child.g + child.h, child.h, next(counter), child))

    return SearchResult("A* (%s)" % heuristic, False, last, len(explored), 0, informed=True)


ALGORITHMS = {"bfs": breadth_first, "dfs": depth_first, "astar": astar}


# --------------------------------------------------------------------------- #
# I/O helpers
# --------------------------------------------------------------------------- #
def read_map(path):
    with open(path) as f:
        lines = [ln.split() for ln in f if ln.strip()]
    rows, cols = int(lines[0][0]), int(lines[0][1])
    grid = [[int(v) for v in lines[1 + i][:cols]] for i in range(rows)]
    start = tuple(int(v) for v in lines[1 + rows][:3])
    goal = tuple(int(v) for v in lines[2 + rows][:3])
    return Problem(grid, start, goal)


def format_state(s):
    return "(%d, %d, %d)" % s


def format_node(node, informed):
    op = node.action if node.action else "None"
    if informed:
        return "(%d, %d, %s, %d, %s)" % (node.depth, node.g, op, node.h, format_state(node.state))
    return "(%d, %d, %s, %s)" % (node.depth, node.g, op, format_state(node.state))


def print_trace(result):
    path = result.node.path()
    print("=== %s ===" % result.algorithm)
    if not result.solved:
        print("No solution found. Path from the initial state to the last examined node:")
    n = len(path) - 1
    for i, node in enumerate(path):
        if i > 0:
            print("Operator %d: %s" % (i, node.action))
        label = "Node %d" % i
        if i == 0:
            label += " (starting node)"
        elif i == n:
            label += " (final node)"
        print("%s: %s" % (label, format_node(node, result.informed)))
    print("Total number of items in explored list: %d" % result.n_explored)
    print("Total number of items in frontier: %d" % result.n_frontier)
    print()
