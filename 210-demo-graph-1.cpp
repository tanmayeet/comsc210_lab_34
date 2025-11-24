// COMSC210 | Lab 34 | Tanmayee Chalamalasetti
// IDE Used: VS Code

#include <iostream>
#include <queue>
#include <vector>
using namespace std;

const int SIZE = 7;

struct Edge {
  int src, dest, weight;
};

typedef pair<int, int>
    Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
 public:
  // a vector of vectors of Pairs to represent an adjacency list
  vector<vector<Pair>> adjList;

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
  vector<Edge> edges = {// (x, y, w) —> edge from x to y having weight w
                        {0, 1, 12}, {0, 2, 8}, {0, 3, 21}, {2, 3, 6}, {2, 6, 2},
                        {5, 6, 6},  {4, 5, 9}, {2, 4, 4},  {2, 5, 5}};

  // Creates graph
  Graph graph(edges);

  // Prints adjacency list representation of graph
  graph.printGraph();

  graph.dfs(0);
  graph.bfs(0);

  return 0;
}
