from dataclasses import dataclass
from typing import List, Dict
from collections import deque


@dataclass
class Edge:
    to: int
    weight: float


class Tree:
    def __init__(self, n: int, root: int):
        self.n = n
        self.root = root
        self.adj = [[] for _ in range(n)]
        self.visited = [False] * n

    def add_edge(self, u: int, v: int, weight: float):
        self.adj[u].append(Edge(v, weight))
        self.adj[v].append(Edge(u, weight))

    def _dfs(self, v: int, rdfs: List[int]):
        self.visited[v] = True
        rdfs.append(v)

        for e in self.adj[v]:
            if not self.visited[e.to]:
                self._dfs(e.to, rdfs)
                rdfs.append(v)  # Return to parent after exploring subtree

    def generate_rdfs(self) -> List[int]:
        self.visited = [False] * self.n
        rdfs = []
        self._dfs(self.root, rdfs)
        return rdfs

    def distance(self, u: int, v: int) -> float:
        if u == v:
            return 0

        vis = [False] * self.n
        dist = [-1] * self.n
        queue = deque([u])
        vis[u] = True
        dist[u] = 0

        while queue:
            s = queue.popleft()
            for e in self.adj[s]:
                if not vis[e.to]:
                    vis[e.to] = True
                    dist[e.to] = dist[s] + e.weight
                    if e.to == v:
                        return dist[e.to]
                    queue.append(e.to)
        return -1

    def path_length(self, path: List[int]) -> float:
        length = 0
        for i in range(1, len(path)):
            for e in self.adj[path[i - 1]]:
                if e.to == path[i]:
                    length += e.weight
                    break
        return length

    def find_path(self, u: int, v: int) -> List[int]:
        if u == v:
            return [u]

        parent = [-1] * self.n
        vis = [False] * self.n
        queue = deque([u])
        vis[u] = True

        while queue:
            s = queue.popleft()
            for e in self.adj[s]:
                if not vis[e.to]:
                    vis[e.to] = True
                    parent[e.to] = s
                    if e.to == v:
                        path = []
                        current = v
                        while current != -1:
                            path.append(current)
                            current = parent[current]
                        return path[::-1]
                    queue.append(e.to)
        return []


@dataclass
class Route:
    vertices: List[int]
    length: float


def calculate_pdfs(tree: Tree, B: float) -> List[Route]:
    routes = []
    rdfs = tree.generate_rdfs()
    root = rdfs[0]
    j_prev = 0

    while j_prev < len(rdfs):
        route = Route(vertices=[], length=0)
        j_current = j_prev

        # Find farthest point we can visit
        for p in range(j_prev + 1, len(rdfs)):
            path_to_j_prev = tree.find_path(root, rdfs[j_prev])
            progress_part = rdfs[j_prev : p + 1]
            path_to_root = tree.find_path(rdfs[p], root)

            total_length = (
                tree.path_length(path_to_j_prev)
                + tree.path_length(progress_part)
                + tree.path_length(path_to_root)
            )

            if total_length <= B:
                j_current = p
            else:
                break

        # Construct route
        path_to_j_prev = tree.find_path(root, rdfs[j_prev])
        progress_part = rdfs[j_prev : j_current + 1]
        path_to_root = tree.find_path(rdfs[j_current], root)

        # Combine parts
        route.vertices = path_to_j_prev
        if path_to_j_prev and progress_part and path_to_j_prev[-1] == progress_part[0]:
            route.vertices.pop()
        route.vertices.extend(progress_part)

        if progress_part and path_to_root and progress_part[-1] == path_to_root[0]:
            route.vertices.pop()
        route.vertices.extend(path_to_root)

        route.length = tree.path_length(route.vertices)
        routes.append(route)

        j_prev = j_current + 1
        if j_current == len(rdfs) - 1:
            break

    return routes


def print_route(route: Route, index: int = None):
    route_str = "R{}: ".format(index) if index is not None else "Route: "
    vertices_str = " ".join(str(v) for v in route.vertices)
    print(f"{route_str}{vertices_str} Length: {route.length}")


def input_tree(tree: Tree, n: int):
    for i in range(n - 1):
        print("Enter edge (u, v) and weight: ", end="")
        u, v, weight = map(float, input().split())
        tree.add_edge(int(u), int(v), weight)


def main():
    print("Enter the number of vertices in the tree: ", end="")
    n = int(input())
    print("Enter the energy budget B: ", end="")
    B = float(input())
    print("Enter the root of the tree: ", end="")
    root = int(input())

    if n <= 0 or B <= 0:
        print("Invalid input. Number of vertices and energy budget must be positive.")
        return

    tree = Tree(n, root)
    print("Input the edges of the tree:")
    input_tree(tree, n)

    pdfs = calculate_pdfs(tree, B)

    print(f"\nPDFS Strategy with B = {B}:")
    for i, route in enumerate(pdfs):
        print_route(route, i + 1)

    print(f"Total number of routes: {len(pdfs)}")


if __name__ == "__main__":
    main()
