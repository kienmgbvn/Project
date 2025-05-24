from dataclasses import dataclass
from typing import List, Tuple, Dict
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
                rdfs.append(v)

    def generate_rdfs(self) -> List[int]:
        self.visited = [False] * self.n
        rdfs = []
        self._dfs(self.root, rdfs)
        return rdfs

    def distance(self, u: int, v: int) -> float:
        if u == v:
            return 0.0

        vis = [False] * self.n
        dist = [-1.0] * self.n
        queue = deque([u])
        vis[u] = True
        dist[u] = 0.0

        while queue:
            s = queue.popleft()
            for e in self.adj[s]:
                if not vis[e.to]:
                    vis[e.to] = True
                    dist[e.to] = dist[s] + e.weight
                    if e.to == v:
                        return dist[e.to]
                    queue.append(e.to)
        return -1.0

    def path_length(self, path: List[int]) -> float:
        length = 0.0
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

    def get_accumulated_weight(self, path: List[int]) -> float:
        total_weight = 0.0
        for i in range(1, len(path)):
            for e in self.adj[path[i - 1]]:
                if e.to == path[i]:
                    total_weight += e.weight
                    break
        return total_weight


@dataclass
class Route:
    vertices: List[int]
    length: float


def calculate_pdfs(tree: Tree, B: float) -> List[Route]:
    routes = []
    rdfs = tree.generate_rdfs()
    root = rdfs[0]
    n = len(rdfs)
    visited = [False] * n
    visited[root] = True

    # Create list of vertices and their weights from root
    vertices = []
    for i in range(n):
        if i != root:
            dist = tree.distance(root, i)
            if dist > 0:  # Only consider connected vertices
                vertices.append((i, dist))

    # Sort vertices by weight
    vertices.sort(key=lambda x: x[1])

    current_route = Route(vertices=[root], length=0.0)

    while vertices:
        current_group = []
        target_weight = vertices[0][1]  # Weight of first unvisited vertex

        # Find vertices with similar weights to group together
        for v, weight in vertices:
            if not visited[v] and weight <= target_weight * 1.5:  # Allow 50% difference
                temp_path = current_route.vertices.copy()

                # Try adding vertex to current route
                for vertex in current_group:
                    path_to_vertex = tree.find_path(temp_path[-1], vertex)
                    temp_path.extend(path_to_vertex[1:])

                path_to_new = tree.find_path(temp_path[-1], v)
                return_to_root = tree.find_path(v, root)

                test_path = temp_path.copy()
                test_path.extend(path_to_new[1:])
                test_path.extend(return_to_root[1:])

                test_length = tree.path_length(test_path)
                if test_length <= B:
                    current_group.append(v)

        if current_group:
            route = Route(vertices=[root], length=0.0)

            # Build route for current group
            for vertex in current_group:
                path_to_vertex = tree.find_path(route.vertices[-1], vertex)
                route.vertices.extend(path_to_vertex[1:])

            # Add return path to root
            if route.vertices[-1] != root:
                return_path = tree.find_path(route.vertices[-1], root)
                route.vertices.extend(return_path[1:])

            route.length = tree.path_length(route.vertices)
            if route.length <= B:
                routes.append(route)
                # Mark visited vertices
                for vertex in current_group:
                    visited[vertex] = True

        # Remove visited vertices from list
        vertices = [(v, w) for v, w in vertices if not visited[v]]

    return routes


def print_route(route: Route, index: int = None):
    route_str = f"R{index}: " if index is not None else "Route: "
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
    for i, route in enumerate(pdfs, 1):
        print_route(route, i)

    print(f"Total number of routes: {len(pdfs)}")


if __name__ == "__main__":
    main()
