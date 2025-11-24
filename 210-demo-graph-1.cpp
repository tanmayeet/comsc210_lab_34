// COMSC210 | Lab 34 | Tanmayee Chalamalasetti
// IDE Used: VS Code

#include <iostream>
#include <queue>
#include <vector>
using namespace std;

const int SIZE = 9;

struct Edge {
  int src, dest, weight;
};

typedef pair<int, int>
    Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
 public:
  // a vector of vectors of Pairs to represent an adjacency list
  vector<vector<Pair>> adjList;

  vector<string> stationNames = {
      "Central Hub",       "Downtown Station",  "University Station",
      "Museum Station",    "City Park Station", "Airport Station",
      "Tech Park Station", "Suburbia Station",  "Stadium Station"};

  // Graph Constructor
  Graph(vector<Edge> const& edges) {
    // resize the vector to hold SIZE elements of type vector<Edge>
    adjList.resize(SIZE);

    // add edges to the directed graph
    for (auto& edge : edges) {
      int src = edge.src;
      int dest = edge.dest;
      int weight = edge.weight;

      // insert at the end
      adjList[src].push_back(make_pair(dest, weight));
      // for an undirected graph, add an edge from dest to src also
      adjList[dest].push_back(make_pair(src, weight));
    }
  }

  // Print the graph's adjacency list
  void printGraph() {
    cout << "Graph's adjacency list:" << endl;
    for (int i = 0; i < adjList.size(); i++) {
      cout << i << " --> ";
      for (Pair v : adjList[i])
        cout << "(" << v.first << ", " << v.second << ") ";
      cout << endl;
    }
  }

  // DFS
  void dfs(int start) {
    vector<bool> visited(adjList.size(), false);
    cout << "DFS starting from vertex " << start << ":" << endl;
    dfsUtil(start, visited);
    cout << endl;
  }

  // BFS
  void bfs(int start) {
    vector<bool> visited(adjList.size(), false);
    queue<int> q;

    visited[start] = true;
    q.push(start);

    cout << "BFS starting from vertex " << start << ":" << endl;

    while (!q.empty()) {
      int v = q.front();
      q.pop();
      cout << v << " ";

      for (Pair neighbor : adjList[v]) {
        int next = neighbor.first;
        if (!visited[next]) {
          visited[next] = true;
          q.push(next);
        }
      }
    }
    cout << endl;
  }

  void printMetroNetwork() {
    cout << "\nCity Metro Network Topology:\n";
    cout << "============================\n";

    for (int i = 0; i < adjList.size(); i++) {
      cout << "Station " << i << " (" << stationNames[i] << ") connects to:\n";
      for (Pair edge : adjList[i]) {
        cout << "  → Station " << edge.first << " (" << stationNames[edge.first]
             << ") - Travel time: " << edge.second << " minutes\n";
      }
    }
  }

  void runDFSInspection(int start) {
    vector<bool> visited(adjList.size(), false);
    cout << "\nMetro Route Inspection (DFS):\n";
    cout << "=============================\n";
    dfsInspectionUtil(start, visited);
  }

  void dfsInspectionUtil(int v, vector<bool>& visited) {
    visited[v] = true;
    cout << "Inspecting Station " << v << " (" << stationNames[v] << ")\n";

    for (Pair edge : adjList[v]) {
      int next = edge.first;
      int travelTime = edge.second;
      if (!visited[next]) {
        cout << "  → Next stop: Station " << next << " (" << stationNames[next]
             << ") - Travel time: " << travelTime << " minutes\n";
        dfsInspectionUtil(next, visited);
      }
    }
  }

  void runBFSCoverage(int start) {
    vector<bool> visited(adjList.size(), false);
    queue<int> q;

    visited[start] = true;
    q.push(start);

    cout << "\nMetro Coverage Analysis (BFS):\n";
    cout << "===============================\n";

    while (!q.empty()) {
      int v = q.front();
      q.pop();

      cout << "Checking Station " << v << " (" << stationNames[v] << ")\n";

      for (Pair edge : adjList[v]) {
        int next = edge.first;
        int travelTime = edge.second;

        if (!visited[next]) {
          visited[next] = true;
          q.push(next);
          cout << "  → Reachable next: Station " << next << " ("
               << stationNames[next] << ") - Travel time: " << travelTime
               << " minutes\n";
        }
      }
    }
  }

 private:
  // DFS helper
  void dfsUtil(int v, vector<bool>& visited) {
    visited[v] = true;
    cout << v << " ";

    for (Pair neighbor : adjList[v]) {
      int next = neighbor.first;
      if (!visited[next]) {
        dfsUtil(next, visited);
      }
    }
  }
};

int main() {
  // Creates a vector of graph edges/weights
  vector<Edge> edges = {{0, 1, 8},  {0, 2, 21}, {1, 2, 6}, {1, 3, 5},
                        {1, 4, 4},  {2, 7, 11}, {2, 8, 8}, {3, 4, 9},
                        {5, 6, 10}, {5, 7, 15}, {5, 8, 5}, {6, 7, 3},
                        {6, 8, 7}};

  // Creates graph
  Graph graph(edges);

  // Prints adjacency list representation of graph
  graph.printGraph();

  graph.dfs(0);
  graph.bfs(0);

  graph.printMetroNetwork();
  graph.runDFSInspection(0);
  graph.runBFSCoverage(0);

  return 0;
}
