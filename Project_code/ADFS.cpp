#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <cmath>
#include <limits>

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

    // Get the diameter of the tree (maximum distance between any two nodes)
    double getDiameter()
    {
        double maxDist = 0;
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                maxDist = max(maxDist, distance(i, j));
            }
        }
        return maxDist;
    }
};

// Structure to represent a route
struct Route
{
    vector<int> vertices;
    double length;
};

// Calculate PDFS strategy with different first route budget
vector<Route> calculatePDFS(Tree &tree, double B, double B_prime)
{
    vector<Route> routes;
    vector<int> rdfs = tree.generateRDFS();
    int root = rdfs[0];

    int j_prev = 0; // Start from the root
    bool isFirstRoute = true;

    while (j_prev < rdfs.size())
    {
        Route route;
        int j_current = j_prev;

        // Use B_prime for the first route, B for subsequent routes
        double currentBudget = isFirstRoute ? B_prime : B;

        // Find the farthest point we can visit with energy budget
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

            if (total_length <= currentBudget)
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

        // No longer the first route
        isFirstRoute = false;

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

// Calculate the cost of a PDFS strategy
double calculatePDFSCost(const vector<Route> &routes, double B_prime, double B)
{
    if (routes.empty())
    {
        return 0.0;
    }

    // For analysis purposes, we assume the first route takes exactly B' time
    // and subsequent routes take exactly B time
    return B_prime + (routes.size() - 1) * B;
}

// Calculate ADFS strategy
pair<double, vector<Route>> calculateADFS(Tree &tree, double B)
{
    double max_cost = 0.0;
    double worst_B_prime = 0.0;
    vector<Route> worst_routes;

    // Get the diameter of the tree as an upper bound for exploration
    double phi_T = tree.getDiameter();

    // Try different values of B' between 0 and min(B, 2*phi_T)
    // In practice, we'd use a more sophisticated approach to find B',
    // but here we'll use a discrete sampling approach for simplicity
    double upper_bound = min(B, 2 * phi_T);
    int num_samples = 100; // Increasing this may improve accuracy

    for (int i = 0; i <= num_samples; i++)
    {
        double B_prime = (i * upper_bound) / num_samples;

        // Calculate PDFS with this B'
        vector<Route> routes = calculatePDFS(tree, B, B_prime);

        // Calculate cost assuming worst case (exactly B' and B lengths)
        double cost = calculatePDFSCost(routes, B_prime, B);

        if (cost > max_cost)
        {
            max_cost = cost;
            worst_B_prime = B_prime;
            worst_routes = routes;
        }
    }

    cout << "Worst B' value: " << worst_B_prime << endl;
    return {max_cost, worst_routes};
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

void inputTree(Tree &tree, int n, int root)
{
    cout << "Enter the edges (u v weight) for the tree:" << endl;
    for (int i = 0; i < n - 1; i++)
    {
        int u, v;
        double weight;
        cin >> u >> v >> weight;
        tree.addEdge(u, v, weight);
    }
}

void inputTree(Tree &tree, int n, int root, vector<pair<int, pair<int, double>>> &edges)
{
    cout << "Enter the edges (u v weight) for the tree:" << endl;
    for (const auto &edge : edges)
    {
        tree.addEdge(edge.first, edge.second.first, edge.second.second);
    }
}

int main()
{
    int n;
    int root;
    double B;
    cout << "Enter the number of vertices in the tree: ";
    cin >> n;
    cout << "Enter the root of the tree: ";
    cin >> root;
    cout << "Enter the budget B: ";
    cin >> B;
    // Create a tree similar to the one in the original code
    Tree tree(n, root);
    inputTree(tree, n, root);

    // Calculate ADFS strategy
    cout << "Calculating ADFS Strategy with B = " << B << ":" << endl;
    pair<double, vector<Route>> adfs_result = calculateADFS(tree, B);
    double adfs_cost = adfs_result.first;
    vector<Route> adfs_routes = adfs_result.second;

    // Print the routes
    cout << "ADFS Strategy with cost: " << adfs_cost << endl;
    for (size_t i = 0; i < adfs_routes.size(); i++)
    {
        cout << "R" << i + 1 << ": ";
        printRoute(adfs_routes[i]);
    }

    cout << "Total number of routes: " << adfs_routes.size() << endl;

    // For comparison, print the original PDFS with B' = B
    cout << "\nFor comparison, PDFS with full budget B = " << B << ":" << endl;
    vector<Route> pdfs_routes = calculatePDFS(tree, B, B);
    for (size_t i = 0; i < pdfs_routes.size(); i++)
    {
        cout << "R" << i + 1 << ": ";
        printRoute(pdfs_routes[i]);
    }
    double pdfs_cost = calculatePDFSCost(pdfs_routes, B, B);
    cout << "PDFS cost: " << pdfs_cost << endl;

    return 0;
}