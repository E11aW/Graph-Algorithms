#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

class Graph
{
private:
  // main components of graph
  int numVertices = 0;
  int numEdges = 0;
  bool directionalEdges;
  vector<vector<int>> adjacencyMatrix;
  unordered_map<string, int> verticesString;
  unordered_map<int, string> verticesInt;

  // used for Kruskal's algorithm
  struct DisjointSet
  {
    map<string, string> previous;
    map<string, int> weight;
    // makes set of vertices
    void makeSet(const string &v)
    {
      previous[v] = v;
      weight[v] = 0;
    }
    // finds vertices
    string find(const string &v)
    {
      if (previous[v] != v)
      {
        previous[v] = find(previous[v]);
      }
      return previous[v];
    }
    // unites vertices as digjionset
    void unite(const string &v1, const string &v2)
    {
      string root1 = find(v1);
      string root2 = find(v2);
      if (root1 != root2)
      {
        if (weight[root1] > weight[root2])
        {
          previous[root2] = root1;
        }
        else if (weight[root1] < weight[root2])
        {
          previous[root1] = root2;
        }
        else
        {
          previous[root2] = root1;
          weight[root1]++;
        }
      }
    }
  };

public:
  // constructor, empty graph
  explicit Graph(bool directionalEdges = true);

  // copy not allowed
  Graph(const Graph &other) = delete;

  // move not allowed
  Graph(Graph &&other) = delete;

  // assignment not allowed
  Graph &operator=(const Graph &other) = delete;

  // move assignment not allowed
  Graph &operator=(Graph &&other) = delete;

  /** destructor, delete all vertices and edges */
  ~Graph();

  // @return true if vertex added, false if it already is in the graph
  bool add(const string &label);

  // @return true if vertex is in the graph
  bool contains(const string &label) const;

  // @return total number of vertices
  int verticesSize() const;

  // Add an edge between two vertices, create new vertices if necessary
  // A vertex cannot connect to itself, cannot have P->P
  // For digraphs (directed graphs), only one directed edge allowed, P->Q
  // Undirected graphs must have P->Q and Q->P with same weight
  // @return true if successfully connected
  bool connect(const string &from, const string &to, int weight = 0);

  // Remove edge from graph
  // @return true if edge successfully deleted
  bool disconnect(const string &from, const string &to);

  // @return total number of edges
  int edgesSize() const;

  // @return number of edges from given vertex, -1 if vertex not found
  int vertexDegree(const string &label) const;

  // @return string representing edges and weights, "" if vertex not found
  // A-3->B, A-5->C should return B(3),C(5)
  string getEdgesAsString(const string &label) const;

  // Read edges from file
  // first line of file is an integer, indicating number of edges
  // each line represents an edge in the form of "string string int"
  // vertex labels cannot contain spaces
  // @return true if file successfully read
  bool readFile(const string &filename);

  // depth-first traversal starting from given startLabel
  void dfs(const string &startLabel, void visit(const string &label));

  // breadth-first traversal starting from startLabel
  // call the function visit on each vertex label */
  void bfs(const string &startLabel, void visit(const string &label));

  // dijkstra's algorithm to find shortest distance to all other vertices
  // and the path to all other vertices
  // Path cost is recorded in the map passed in, e.g. weight["F"] = 10
  // How to get to the vertex is recorded previous["F"] = "C"
  // @return a pair made up of two map objects, Weights and Previous
  pair<map<string, int>, map<string, string>>
  dijkstra(const string &startLabel) const;

  // minimum spanning tree using Prim's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstPrim(const string &startLabel,
              void visit(const string &from, const string &to,
                         int weight)) const;

  // minimum spanning tree using Kruskal's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstKruskal(void visit(const string &from, const string &to,
                            int weight)) const;
};

#endif // GRAPH_H
