from typing import List, Tuple, Dict
from dataclasses import dataclass
import math


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
        rdfs = []
        self.visited = [False] * self.n
        self._dfs(self.root, rdfs)
        return rdfs

    def distance(self, u: int, v: int) -> float:
        if u == v:
            return 0

        vis = [False] * self.n
        dist = [-1] * self.n
        queue = []

        vis[u] = True
        dist[u] = 0
        queue.append(u)

        while queue:
            s = queue.pop(0)
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
        queue = []

        vis[u] = True
        queue.append(u)

        while queue:
            s = queue.pop(0)
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

    def get_diameter(self) -> float:
        max_dist = 0
        for i in range(self.n):
            for j in range(i + 1, self.n):
                max_dist = max(max_dist, self.distance(i, j))
        return max_dist


@dataclass
class Route:
    vertices: List[int]
    length: float


def calculate_pdfs(tree: Tree, B: float, B_prime: float) -> List[Route]:
    routes = []
    rdfs = tree.generate_rdfs()
    root = rdfs[0]

    j_prev = 0
    is_first_route = True

    while j_prev < len(rdfs):
        j_current = j_prev
        current_budget = B_prime if is_first_route else B

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

            if total_length <= current_budget:
                j_current = p
            else:
                break

        # Construct route
        route = Route(vertices=[], length=0)
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

        is_first_route = False
        j_prev = j_current + 1

        if j_current == len(rdfs) - 1:
            break

    return routes


def calculate_pdfs_cost(routes: List[Route], B_prime: float, B: float) -> float:
    if not routes:
        return 0.0
    return B_prime + (len(routes) - 1) * B


def calculate_adfs(tree: Tree, B: float) -> Tuple[float, List[Route]]:
    max_cost = 0.0
    worst_B_prime = 0.0
    worst_routes = []

    phi_T = tree.get_diameter()
    upper_bound = min(B, 2 * phi_T)
    num_samples = 100

    for i in range(num_samples + 1):
        B_prime = (i * upper_bound) / num_samples
        routes = calculate_pdfs(tree, B, B_prime)
        cost = calculate_pdfs_cost(routes, B_prime, B)

        if cost > max_cost:
            max_cost = cost
            worst_B_prime = B_prime
            worst_routes = routes

    print(f"Worst B' value: {worst_B_prime}")
    return max_cost, worst_routes


def print_route(route: Route, index: int = None):
    route_str = "R{}: ".format(index) if index is not None else "Route: "
    vertices_str = " ".join(str(v) for v in route.vertices)
    print(f"{route_str}{vertices_str} Length: {route.length}")


def main():
    n = int(input("Enter the number of vertices in the tree: "))
    root = int(input("Enter the root of the tree: "))
    B = float(input("Enter the budget B: "))

    tree = Tree(n, root)
    print("Enter the edges (u v weight) for the tree:")
    for _ in range(n - 1):
        u, v, weight = map(float, input().split())
        tree.add_edge(int(u), int(v), weight)

    print(f"\nCalculating ADFS Strategy with B = {B}:")
    adfs_cost, adfs_routes = calculate_adfs(tree, B)

    print(f"ADFS Strategy with cost: {adfs_cost}")
    for i, route in enumerate(adfs_routes):
        print_route(route, i + 1)
    print(f"Total number of routes: {len(adfs_routes)}")

    print(f"\nFor comparison, PDFS with full budget B = {B}:")
    pdfs_routes = calculate_pdfs(tree, B, B)
    for i, route in enumerate(pdfs_routes):
        print_route(route, i + 1)
    pdfs_cost = calculate_pdfs_cost(pdfs_routes, B, B)
    print(f"PDFS cost: {pdfs_cost}")


if __name__ == "__main__":
    main()
