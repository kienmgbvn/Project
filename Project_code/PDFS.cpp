#include <iostream>
#include <vector>
#include <list>
#include <algorithm>

using namespace std;

// Structure to represent an edge in the tree
struct Edge
{
    int to;
    double weight;
};

// Structure to represent a tree
class Tree
{
private:
    int n;                    // Number of vertices
    int root;                 // Root of the tree
    vector<vector<Edge>> adj; // Adjacency list
    vector<bool> visited;     // To track visited nodes during DFS

    // Helper function for DFS traversal
    void dfs(int v, vector<int> &rdfs)
    {
        visited[v] = true;
        rdfs.push_back(v);

        for (const Edge &e : adj[v])
        {
            if (!visited[e.to])
            {
                dfs(e.to, rdfs);
                rdfs.push_back(v); // Return to parent after exploring subtree
            }
        }
    }

public:
    Tree(int n, int root) : n(n), root(root)
    {
        adj.resize(n);
        visited.resize(n, false);
    }

    // Add an undirected edge to the tree
    void addEdge(int u, int v, double weight)
    {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight});
    }

    // Generate the RDFS traversal
    vector<int> generateRDFS()
    {
        vector<int> rdfs;
        fill(visited.begin(), visited.end(), false);
        dfs(root, rdfs);
        return rdfs;
    }

    // Calculate distance between two vertices
    double distance(int u, int v)
    {
        if (u == v)
            return 0;

        vector<bool> vis(n, false);
        vector<double> dist(n, -1);
        list<int> queue;

        vis[u] = true;
        dist[u] = 0;
        queue.push_back(u);

        while (!queue.empty())
        {
            int s = queue.front();
            queue.pop_front();

            for (const Edge &e : adj[s])
            {
                if (!vis[e.to])
                {
                    vis[e.to] = true;
                    dist[e.to] = dist[s] + e.weight;

                    if (e.to == v)
                        return dist[e.to];

                    queue.push_back(e.to);
                }
            }
        }

        return -1; // Should not reach here in a connected tree
    }

    // Calculate the length of a path
    double pathLength(const vector<int> &path)
    {
        double length = 0;
        for (size_t i = 1; i < path.size(); i++)
        {
            // Find the edge weight between path[i-1] and path[i]
            for (const Edge &e : adj[path[i - 1]])
            {
                if (e.to == path[i])
                {
                    length += e.weight;
                    break;
                }
            }
        }
        return length;
    }

    // Find the path from u to v
    vector<int> findPath(int u, int v)
    {
        if (u == v)
            return {u};

        vector<int> parent(n, -1);
        vector<bool> vis(n, false);
        list<int> queue;

        vis[u] = true;
        queue.push_back(u);

        while (!queue.empty())
        {
            int s = queue.front();
            queue.pop_front();

            for (const Edge &e : adj[s])
            {
                if (!vis[e.to])
                {
                    vis[e.to] = true;
                    parent[e.to] = s;

                    if (e.to == v)
                    {
                        // Construct the path
                        vector<int> path;
                        int current = v;
                        while (current != -1)
                        {
                            path.push_back(current);
                            current = parent[current];
                        }
                        reverse(path.begin(), path.end());
                        return path;
                    }

                    queue.push_back(e.to);
                }
            }
        }

        return {}; // Should not reach here in a connected tree
    }
};

// Structure to represent a route
struct Route
{
    vector<int> vertices;
    double length;
};

// Calculate PDFS strategy
vector<Route> calculatePDFS(Tree &tree, double B)
{
    vector<Route> routes;
    vector<int> rdfs = tree.generateRDFS();
    int root = rdfs[0];

    int j_prev = 0; // Start from the root

    while (j_prev < rdfs.size())
    {
        Route route;
        int j_current = j_prev;

        // Find the farthest point we can visit with energy budget B
        for (int p = j_prev + 1; p < rdfs.size(); p++)
        {
            // Calculate the tentative route: root -> j_prev -> p -> root
            vector<int> path_to_j_prev = tree.findPath(root, rdfs[j_prev]);

            // The progress part is simply the subarray of rdfs from j_prev to p
            vector<int> progress_part;
            for (int idx = j_prev; idx <= p; idx++)
            {
                progress_part.push_back(rdfs[idx]);
            }

            vector<int> path_to_root = tree.findPath(rdfs[p], root);

            // Calculate total length
            double total_length = tree.pathLength(path_to_j_prev) +
                                  tree.pathLength(progress_part) +
                                  tree.pathLength(path_to_root);

            if (total_length <= B)
            {
                j_current = p;
            }
            else
            {
                break; // Exceeds budget, stop here
            }
        }

        // Construct the route
        vector<int> path_to_j_prev = tree.findPath(root, rdfs[j_prev]);

        // The progress part
        vector<int> progress_part;
        for (int idx = j_prev; idx <= j_current; idx++)
        {
            progress_part.push_back(rdfs[idx]);
        }

        vector<int> path_to_root = tree.findPath(rdfs[j_current], root);

        // Combine the parts to form the complete route
        route.vertices = path_to_j_prev;
        // Remove the last element to avoid duplication
        if (!path_to_j_prev.empty() && !progress_part.empty() &&
            path_to_j_prev.back() == progress_part.front())
        {
            route.vertices.pop_back();
        }
        route.vertices.insert(route.vertices.end(), progress_part.begin(), progress_part.end());

        // Remove the last element to avoid duplication
        if (!progress_part.empty() && !path_to_root.empty() &&
            progress_part.back() == path_to_root.front())
        {
            route.vertices.pop_back();
        }
        route.vertices.insert(route.vertices.end(), path_to_root.begin(), path_to_root.end());

        route.length = tree.pathLength(route.vertices);
        routes.push_back(route);

        // Move to the next starting point
        j_prev = j_current + 1;

        // If we've reached the end of rdfs, we're done
        if (j_current == rdfs.size() - 1)
        {
            break;
        }
    }

    return routes;
}

// Print a route
void printRoute(const Route &route)
{
    cout << "Route: ";
    for (int vertex : route.vertices)
    {
        cout << vertex << " ";
    }
    cout << "Length: " << route.length << endl;
}

void inputTree(Tree &tree, int n)
{
    for (int i = 0; i < n - 1; i++)
    {
        int u, v;
        double weight;
        cout << "Enter edge (u, v) and weight: ";
        cin >> u >> v >> weight;
        tree.addEdge(u, v, weight);
    }
}

int main()
{
    int n; // Number of vertices
    int root;
    double B; // Energy budget
    cout << "Enter the number of vertices in the tree: ";
    cin >> n;
    cout << "Enter the energy budget B: ";
    cin >> B;
    cout << "Enter the root of the tree: ";
    cin >> root;
    if (n <= 0 || B <= 0)
    {
        cerr << "Invalid input. Number of vertices and energy budget must be positive." << endl;
        return 1;
    }
    // Create a tree similar to the one in Fig.1 of the problem statement
    Tree tree(n, root);
    cout << "Input the edges of the tree:" << endl;
    inputTree(tree, n);

    vector<Route> pdfs = calculatePDFS(tree, B);

    // Print the routes
    cout << "PDFS Strategy with B = " << B << ":" << endl;
    for (size_t i = 0; i < pdfs.size(); i++)
    {
        cout << "R" << i + 1 << ": ";
        printRoute(pdfs[i]);
    }

    cout << "Total number of routes: " << pdfs.size() << endl;
}