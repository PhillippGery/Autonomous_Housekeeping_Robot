# Extracted from monolithic nodes. No logic changes.

import numpy as np
from copy import copy
from queue import PriorityQueue


class Node():
    """Graph node holding a name, child nodes, and edge weights."""

    def __init__(self, name):
        """Initialise a node with the given name and empty child/weight lists."""
        self.name = name
        self.children = []
        self.weight = []

    def __repr__(self):
        """Return the node name as its string representation."""
        return self.name

    def add_children(self, node, w=None):
        """Append child nodes and corresponding edge weights to this node."""
        if w is None:
            w = [1] * len(node)
        self.children.extend(node)
        self.weight.extend(w)


class Tree():
    """Directed graph (adjacency dict) used for grid-based path planning."""

    def __init__(self, name):
        """Initialise an empty tree with the given name."""
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
        return

    def add_node(self, node, start=False, end=False):
        """Add a node to the graph dict; optionally mark it as root or end."""
        self.g[node.name] = node
        if start:
            self.root = node.name
        elif end:
            self.end = node.name

    def set_as_root(self, node):
        """Mark the given node as the root of the tree."""
        self.root = True
        self.end = False

    def set_as_end(self, node):
        """Mark the given node as the end of the tree."""
        self.root = False
        self.end = True


class Queue():
    """Simple FIFO queue backed by a Python list."""

    def __init__(self, init_queue=[]):
        """Initialise the queue, optionally with an existing list."""
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue) - 1

    def __len__(self):
        """Return the number of elements in the queue."""
        numel = len(self.queue)
        return numel

    def __repr__(self):
        """Return a human-readable representation of the queue."""
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if i == self.start:
                tmpstr += "<"
                flag = True
            if i == self.end:
                tmpstr += ">"
                flag = True
            if flag:
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'
        return tmpstr

    def __call__(self):
        """Return the underlying queue list."""
        return self.queue

    def initialize_queue(self, init_queue=[]):
        """Replace the queue contents with init_queue."""
        self.queue = copy(init_queue)

    def sort(self, key=str.lower):
        """Sort the queue in-place using the given key function."""
        self.queue = sorted(self.queue, key=key)

    def push(self, data):
        """Append data to the end of the queue."""
        self.queue.append(data)
        self.end += 1

    def pop(self):
        """Remove and return the front element of the queue."""
        p = self.queue.pop(self.start)
        self.end = len(self.queue) - 1
        return p


class AStar():
    """A* shortest-path solver operating on a Tree graph."""

    def __init__(self, in_tree):
        """Initialise A* with the given Tree; sets up dist, h, and via dicts."""
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name: np.inf for name, node in in_tree.g.items()}
        self.h = {name: 0 for name, node in in_tree.g.items()}
        self.via = {name: None for name in in_tree.g}
        for __, node in in_tree.g.items():
            self.q.push(node)

    def __get_f_score(self, node):
        """Return f = g + h for the named node."""
        return self.dist[node] + self.h[node]

    def solve(self, sn, en):
        """Run A* from start node sn to end node en; return (path_names, dist)."""
        open_set = PriorityQueue()

        entry_count = 0
        self.dist[sn.name] = 0
        start_f_score = self.__get_f_score(sn.name)

        open_set.put((start_f_score, entry_count, sn))
        entry_count += 1

        while not open_set.empty():
            current_node = open_set.get()[2]

            if current_node.name == en.name:
                return self.reconstruct_path(sn.name, en.name)

            for i, child_node in enumerate(current_node.children):
                weight = current_node.weight[i]
                tentative_g_score = self.dist[current_node.name] + weight

                if tentative_g_score < self.dist[child_node.name]:
                    self.via[child_node.name] = current_node.name
                    self.dist[child_node.name] = tentative_g_score
                    f_score = self.__get_f_score(child_node.name)
                    open_set.put((f_score, entry_count, child_node))
                    entry_count += 1

        return [], np.inf

    def reconstruct_path(self, sn, en):
        """Reconstruct the path from start name sn to end name en."""
        end_name = en.name if hasattr(en, 'name') else en
        path = []
        dist = self.dist[end_name]

        current = end_name
        while current is not None:
            path.append(current)
            current = self.via[current]

        return path[::-1], dist
