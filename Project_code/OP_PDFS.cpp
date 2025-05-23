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

    // Calculate the accumulated weight for a path
    double getAccumulatedWeight(const vector<int> &path)
    {
        double totalWeight = 0;
        for (size_t i = 1; i < path.size(); i++)
        {
            for (const Edge &e : adj[path[i - 1]])
            {
                if (e.to == path[i])
                {
                    totalWeight += e.weight;
                    break;
                }
            }
        }
        return totalWeight;
    }
};

// Structure to represent a route
struct Route
{
    vector<int> vertices;
    double length;
};

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

// Calculate PDFS strategy
vector<Route> calculatePDFS(Tree &tree, double B)
{
    vector<Route> routes;
    vector<int> rdfs = tree.generateRDFS();
    int root = rdfs[0];
    int n = rdfs.size();
    vector<bool> visited(n, false);
    visited[root] = true;

    // Tạo danh sách các đỉnh và trọng số tương ứng từ root
    vector<pair<int, double>> vertices;
    for (int i = 0; i < n; i++)
    {
        if (i != root)
        {
            double dist = tree.distance(root, i);
            if (dist > 0) // Chỉ xét các đỉnh có kết nối
            {
                vertices.push_back({i, dist});
            }
        }
    }

    // Sắp xếp các đỉnh theo trọng số tăng dần
    sort(vertices.begin(), vertices.end(),
         [](const pair<int, double> &a, const pair<int, double> &b)
         {
             return a.second < b.second;
         });

    Route currentRoute;
    currentRoute.vertices = {root};
    double currentLength = 0;

    // Nhóm các đỉnh theo trọng số để tạo các tuyến đường
    while (!vertices.empty())
    {
        vector<int> currentGroup;
        double targetWeight = vertices[0].second; // Trọng số của đỉnh đầu tiên chưa thăm

        // Tìm các đỉnh có trọng số gần bằng nhau để gom vào một nhóm
        for (const auto &v : vertices)
        {
            if (!visited[v.first] && v.second <= targetWeight * 1.5) // Độ chênh lệch cho phép 50%
            {
                vector<int> tempPath = currentRoute.vertices;

                // Thử thêm đỉnh vào tuyến đường hiện tại
                for (int vertex : currentGroup)
                {
                    vector<int> pathToVertex = tree.findPath(tempPath.back(), vertex);
                    for (size_t i = 1; i < pathToVertex.size(); i++)
                    {
                        tempPath.push_back(pathToVertex[i]);
                    }
                }

                vector<int> pathToNew = tree.findPath(tempPath.back(), v.first);
                vector<int> returnToRoot = tree.findPath(v.first, root);

                vector<int> testPath = tempPath;
                for (size_t i = 1; i < pathToNew.size(); i++)
                {
                    testPath.push_back(pathToNew[i]);
                }
                for (size_t i = 1; i < returnToRoot.size(); i++)
                {
                    testPath.push_back(returnToRoot[i]);
                }

                double testLength = tree.pathLength(testPath);
                if (testLength <= B)
                {
                    currentGroup.push_back(v.first);
                }
            }
        }

        if (!currentGroup.empty())
        {
            Route route;
            route.vertices = {root};

            // Xây dựng tuyến đường cho nhóm hiện tại
            for (int vertex : currentGroup)
            {
                vector<int> pathToVertex = tree.findPath(route.vertices.back(), vertex);
                for (size_t i = 1; i < pathToVertex.size(); i++)
                {
                    route.vertices.push_back(pathToVertex[i]);
                }
            }

            // Thêm đường về root
            if (route.vertices.back() != root)
            {
                vector<int> returnPath = tree.findPath(route.vertices.back(), root);
                for (size_t i = 1; i < returnPath.size(); i++)
                {
                    route.vertices.push_back(returnPath[i]);
                }
            }

            route.length = tree.pathLength(route.vertices);
            if (route.length <= B)
            {
                routes.push_back(route);
                // Đánh dấu các đỉnh đã thăm
                for (int vertex : currentGroup)
                {
                    visited[vertex] = true;
                }
            }
        }

        // Loại bỏ các đỉnh đã thăm khỏi danh sách
        vertices.erase(
            remove_if(vertices.begin(), vertices.end(),
                      [&visited](const pair<int, double> &v)
                      { return visited[v.first]; }),
            vertices.end());
    }

    return routes;
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