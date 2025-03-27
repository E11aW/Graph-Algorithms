#include "graph.h"
#include <algorithm>
#include <climits>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  this->directionalEdges = directionalEdges;
}

// destructor
Graph::~Graph() {} // no destructor necessary

// @return total number of vertices
int Graph::verticesSize() const { return numVertices; }

// @return total number of edges
int Graph::edgesSize() const { return numEdges; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (!contains(label)) {
    return -1; // label not in matrix
  }
  int currNumEdges = 0;
  int currVertex = verticesString.at(label);
  for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
    if (adjacencyMatrix[currVertex][i] != 0) {
      currNumEdges++;
    }
  }
  return currNumEdges;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    return false;
  }
  verticesString[label] = numVertices;
  verticesInt[numVertices] = label;
  numVertices++;
  adjacencyMatrix.resize(numVertices);
  for (auto &row : adjacencyMatrix) {
    row.resize(numVertices, 0);
  }
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return verticesString.find(label) != verticesString.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  if (!contains(label)) {
    return "";
  }
  map<int, string> order;
  int currVertex = verticesString.at(label);
  for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
    int weight = adjacencyMatrix[currVertex][i];
    if (weight != 0) {
      order[weight] = verticesInt.at(i);
    }
  }
  if (order.empty()) {
    return "";
  }
  string edges;
  for (auto &vertex : order) {
    edges += vertex.second + "(" + to_string(vertex.first) + ")" + ",";
  }
  edges.pop_back();
  return edges;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (!contains(from)) {
    add(from);
  }
  if (!contains(to)) {
    add(to);
  }
  int fromVertex = verticesString[from];
  int toVertex = verticesString[to];
  if (from == to ||
      (directionalEdges && adjacencyMatrix[toVertex][fromVertex] == weight) ||
      adjacencyMatrix[fromVertex][toVertex] != 0) {
    return false;
  }
  adjacencyMatrix[fromVertex][toVertex] = weight;
  numEdges++;
  // adds additional edge if undirected
  if (!directionalEdges) {
    adjacencyMatrix[toVertex][fromVertex] = weight;
    numEdges++;
  }
  return true;
}

bool Graph::disconnect(const string &from, const string &to) {
  if (!contains(from) || !contains(to) || from == to) {
    return false;
  }
  int fromVertex = verticesString[from];
  int toVertex = verticesString[to];
  if (adjacencyMatrix[fromVertex][toVertex] == 0) {
    return false;
  }
  adjacencyMatrix[fromVertex][toVertex] = 0;
  numEdges--;
  // removes additional edge if not directed
  if (!directionalEdges) {
    adjacencyMatrix[toVertex][fromVertex] = 0;
    numEdges--;
  }
  return true;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  unordered_set<string> visited;
  stack<string> st;
  st.push(startLabel);
  visited.insert(startLabel);

  while (!st.empty()) {
    string currLabel = st.top();
    st.pop();
    visit(currLabel);
    int currVertex = verticesString.at(currLabel);
    set<string, greater<string>> neighbors;
    for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
      int currWeight = adjacencyMatrix[currVertex][i];
      if (currWeight != 0) {
        neighbors.insert(verticesInt.at(i));
      }
    }
    // enusres neighbors are visited in the ocrrect order
    for (const auto &neighbor : neighbors) {
      if (visited.find(neighbor) == visited.end()) {
        st.push(neighbor);
        visited.insert(neighbor);
      }
    }
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  unordered_set<string> visited;
  queue<string> q;
  q.push(startLabel);
  visited.insert(startLabel);

  while (!q.empty()) {
    string currLabel = q.front();
    q.pop();
    visit(currLabel);
    int currVertex = verticesString.at(currLabel);
    set<string> neighbors;
    for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
      int currWeight = adjacencyMatrix[currVertex][i];
      if (currWeight != 0) {
        neighbors.insert(verticesInt.at(i));
      }
    }
    // enusres neighbors are visited in the ocrrect order
    for (const auto &neighbor : neighbors) {
      if (visited.find(neighbor) == visited.end()) {
        q.push(neighbor);
        visited.insert(neighbor);
      }
    }
  }
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  unordered_set<string> visited;

  if (!contains(startLabel)) {
    return make_pair(weights, previous);
  }

  priority_queue<pair<int, string>, vector<pair<int, string>>,
                 greater<pair<int, string>>>
      pq;
  pq.push({0, startLabel});
  while (!pq.empty()) {
    string currLabel = pq.top().second;
    int currDistance = pq.top().first;
    pq.pop();
    if (visited.find(currLabel) == visited.end()) {
      visited.insert(currLabel);
      int currVertex = verticesString.at(currLabel);
      for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
        int edgeWeight = adjacencyMatrix[currVertex][i];
        if (edgeWeight != 0) {
          string currNeighbor = verticesInt.at(i);
          if (visited.find(currNeighbor) == visited.end()) {
            int newDistance = currDistance + edgeWeight;
            if (weights[currNeighbor] == 0) {
              weights[currNeighbor] = INT_MAX;
            }
            if (newDistance < weights[currNeighbor]) {
              weights[currNeighbor] = newDistance;
              previous[currNeighbor] = currLabel;
              pq.push({newDistance, currNeighbor});
            }
          }
        }
      }
    }
  }
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    return -1;
  }
  // implemented using similar logic to dijkstra's algorithm
  map<string, int> weights;
  map<string, string> previous;
  unordered_set<string> visited;
  int edgeLength = 0;
  priority_queue<pair<int, string>, vector<pair<int, string>>,
                 greater<pair<int, string>>>
      pq;
  pq.push({0, startLabel});
  while (!pq.empty()) {
    string currLabel = pq.top().second;
    pq.pop();
    if (visited.find(currLabel) == visited.end()) {
      if (currLabel != startLabel) {
        visit(previous[currLabel], currLabel, weights[currLabel]);
      }
      visited.insert(currLabel);
      edgeLength += weights[currLabel];
      int currVertex = verticesString.at(currLabel);
      for (int i = 0; i < adjacencyMatrix[currVertex].size(); i++) {
        int edgeWeight = adjacencyMatrix[currVertex][i];
        if (edgeWeight != 0) {
          string currNeighbor = verticesInt.at(i);
          if (visited.find(currNeighbor) == visited.end()) {
            if (weights[currNeighbor] == 0) {
              weights[currNeighbor] = INT_MAX;
            }
            if (edgeWeight < weights[currNeighbor]) {
              weights[currNeighbor] = edgeWeight;
              previous[currNeighbor] = currLabel;
              pq.push({edgeWeight, currNeighbor});
            }
          }
        }
      }
    }
  }
  return edgeLength;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  if (numVertices == 0) {
    return -1;
  }
  struct Edge {
    string previous, current;
    int weight;
  };
  vector<Edge> edges;
  for (int i = 0; i < numVertices; i++) {
    for (int j = i + 1; j < numVertices; j++) {
      int edgeWeight = adjacencyMatrix[i][j];
      if (edgeWeight != 0) {
        // ensures vertices added alphabetically
        string u = verticesInt.at(i);
        string v = verticesInt.at(j);
        if (u > v) {
          swap(u, v);
        }
        edges.push_back({u, v, edgeWeight});
      }
    }
  }

  sort(edges.begin(), edges.end(),
       [](Edge a, Edge b) { return a.weight < b.weight; });

  DisjointSet dset;
  for (const auto &v : verticesString) {
    dset.makeSet(v.first);
  }
  // visits vertices by edge, ensuring no cycles
  int totalEdgeLength = 0;
  for (const auto &edge : edges) {
    if (dset.find(edge.previous) != dset.find(edge.current)) {
      dset.unite(edge.previous, edge.current);
      visit(edge.previous, edge.current, edge.weight);
      totalEdgeLength += edge.weight;
    }
  }
  return totalEdgeLength;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}