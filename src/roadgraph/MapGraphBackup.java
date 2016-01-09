/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *         <p>
 *         A class which represents a graph of geographic locations
 *         Nodes in the graph are intersections between
 */
public class MapGraphBackup {
  //TODO: Add your member variables here in WEEK 2
  private Map<GeographicPoint, Vertex> vertices;


  /**
   * Create a new empty MapGraph
   */
  public MapGraphBackup() {
    // TODO: Implement in this constructor in WEEK 2
    vertices = new HashMap<>();
  }

  /**
   * Get the number of vertices (road intersections) in the graph
   *
   * @return The number of vertices in the graph.
   */
  public int getNumVertices() {
    //TODO: Implement this method in WEEK 2
    return vertices.size();
  }

  /**
   * Return the intersections, which are the vertices in this graph.
   *
   * @return The vertices in this graph as GeographicPoints
   */
  public Set<GeographicPoint> getVertices() {
    //TODO: Implement this method in WEEK 2
    return vertices.keySet();
  }

  /**
   * Get the number of road segments in the graph
   *
   * @return The number of edges in the graph.
   */
  public int getNumEdges() {
    //TODO: Implement this method in WEEK 2
    Set<Edge> edges = new HashSet<>();
    for (Vertex vertex : vertices.values()) {
      edges.addAll(vertex.getNeighbours());
    }
    return edges.size();
  }


  /**
   * Add a node corresponding to an intersection at a Geographic Point
   * If the location is already in the graph or null, this method does
   * not change the graph.
   *
   * @param location The location of the intersection
   * @return true if a node was added, false if it was not (the node
   * was already in the graph, or the parameter is null).
   */
  public boolean addVertex(GeographicPoint location) {
    // TODO: Implement this method in WEEK 2
    boolean addedVertex = false;
    if (location != null && !vertices.containsKey(location)) {
      vertices.put(location, new Vertex(location));
      addedVertex = true;
    }
    return addedVertex;
  }

  /**
   * Adds a directed edge to the graph from pt1 to pt2.
   * Precondition: Both GeographicPoints have already been added to the graph
   *
   * @param from     The starting point of the edge
   * @param to       The ending point of the edge
   * @param roadName The name of the road
   * @param roadType The type of the road
   * @param length   The length of the road, in km
   * @throws IllegalArgumentException If the points have not already been
   *                                  added as nodes to the graph, if any of the arguments is null,
   *                                  or if the length is less than 0.
   */
  public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                      String roadType, double length) throws IllegalArgumentException {
    //TODO: Implement this method in WEEK 2
    boolean error = false;
    String errorMessage = null;
    if (from == null || to == null || roadName == null || roadType == null) {
      error = true;
      errorMessage = "Arguments cannot be null.";
    } else if (length < 0) {
      error = true;
      errorMessage = "length cannot be less than 0.";
    } else if (!vertices.containsKey(from) || !vertices.containsKey(to)) {
      error = true;
      errorMessage = "Both GeographicPoints must have already been added to the graph.";
    }
    if (error) {
      throw new IllegalArgumentException(errorMessage);
    }
    vertices.get(from).addNeighbour(new Edge(from, to, roadName, roadType, length));
  }


  /**
   * Find the path from start to goal using breadth first search
   *
   * @param start The starting location
   * @param goal  The goal location
   * @return The list of intersections that form the shortest (unweighted)
   * path from start to goal (including both start and goal).
   */
  public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
    // Dummy variable for calling the search algorithms
    Consumer<GeographicPoint> temp = (x) -> {
    };
    return bfs(start, goal, temp);
  }

  /**
   * Find the path from start to goal using breadth first search
   *
   * @param start        The starting location
   * @param goal         The goal location
   * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
   * @return The list of intersections that form the shortest (unweighted)
   * path from start to goal (including both start and goal).
   */
  public List<GeographicPoint> bfs(GeographicPoint start,
                                   GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
    // TODO: Implement this method in WEEK 2

    Vertex startNode, endNode;
    if (start == null) {                                          // Validate method arguments
      throw new IllegalArgumentException("Start node is null!");
    } else {
      startNode = vertices.get(start);
    }

    if (goal == null) {
      throw new IllegalArgumentException("Goal node is null!");
    } else {
      endNode = vertices.get(goal);
    }

    // Initialize: queue, visited HashSet and parent HashMap
    Set<Vertex> visitedSet = new HashSet<>();
    Queue<Vertex> unvisitedQueue = new LinkedList<>();
    Map<Vertex, Vertex> parentMap = new HashMap<>();

    // Enqueue starNode onto the queue and add to visited
    unvisitedQueue.add(startNode);
    visitedSet.add(startNode);

    // Check whether path was found via helper method
    if (!bfsSearch(unvisitedQueue, visitedSet, parentMap, endNode, nodeSearched)) {
      System.err.println("No path found");
      return null;
    }

    // We know that a path was found, so we can return it via another helper method
    return createPath(startNode, endNode, parentMap);
  }


  /**
   * Helper method to print vertex information for debugging
   */
  private void printVertices(){
    for (Vertex vertex : vertices.values()) {
      System.out.println(vertex);
    }
  }

  /**
   * Helper method to print the node's neighbours
   */
  private void printNeighbours() {
    for (Vertex vertex : vertices.values()) {
      System.out.println(vertex.getNeighbours());
    }
  }

  /**
   * Helper Method containing Breadth First Search algorithm
   *
   * @param unvisitedQueue
   * @param visitedSet
   * @param parentMap
   * @param endNode
   * @param nodeSearched
   * @return result determining whether path was found via Breadth First Search
   */
  private boolean bfsSearch(Queue<Vertex> unvisitedQueue, Set<Vertex> visitedSet, Map<Vertex, Vertex> parentMap,
                            Vertex endNode, Consumer<GeographicPoint> nodeSearched) {

    boolean result = false;

    while (!unvisitedQueue.isEmpty()) {                     // while queue is not empty:
      Vertex current = unvisitedQueue.poll();               // dequeue current node from front of queue
      if (current == endNode) {
        result = true;                                       // set search result to true if match is found
      }
      Iterator<Edge> edgeIterator = current.getNeighbours().listIterator();
      while (edgeIterator.hasNext()) {
        Vertex next = vertices.get(edgeIterator.next().getDestination());

        nodeSearched.accept(next.getLocation());            // Hook for visualization.  See writeup.

        if (!visitedSet.contains(next)) {                   // for each of current's neighbours, n, not in visited set:
          visitedSet.add(next);                             // add n to visited set
          unvisitedQueue.add(next);                         // enqueue n onto the queue
          parentMap.put(next, current);                     // add current as n's parent in parent map
        }
      }                                                     // If we get here then there's no path,
    }                                                       // i.e. result is set to false
    return result;
  }

  /**
   * Helper method to create final path
   * @param startNode
   * @param endNode
   * @param parentMap
   * @return Breadth First Search path list of GeographicPoint
   */
  private List<GeographicPoint> createPath(Vertex startNode, Vertex endNode, Map<Vertex, Vertex> parentMap) {
    LinkedList<GeographicPoint> path = new LinkedList<>();
    Vertex current = endNode;
    while (current != startNode) {
      path.addFirst(current.getLocation());
      current = parentMap.get(current);
    }
    // add start
    path.addFirst(startNode.getLocation());
    return path;
  }

  /**
   * Find the path from start to goal using Dijkstra's algorithm
   *
   * @param start The starting location
   * @param goal  The goal location
   * @return The list of intersections that form the shortest path from
   * start to goal (including both start and goal).
   */
  public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
    // Dummy variable for calling the search algorithms
    // You do not need to change this method.
    Consumer<GeographicPoint> temp = (x) -> {
    };
    return dijkstra(start, goal, temp);
  }

  /**
   * Find the path from start to goal using Dijkstra's algorithm
   *
   * @param start        The starting location
   * @param goal         The goal location
   * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
   * @return The list of intersections that form the shortest path from
   * start to goal (including both start and goal).
   */
  public List<GeographicPoint> dijkstra(GeographicPoint start,
                                        GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
    // TODO: Implement this method in WEEK 3

    // Hook for visualization.  See writeup.
    //nodeSearched.accept(next.getLocation());

    return null;
  }

  /**
   * Find the path from start to goal using A-Star search
   *
   * @param start The starting location
   * @param goal  The goal location
   * @return The list of intersections that form the shortest path from
   * start to goal (including both start and goal).
   */
  public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
    // Dummy variable for calling the search algorithms
    Consumer<GeographicPoint> temp = (x) -> {
    };
    return aStarSearch(start, goal, temp);
  }

  /**
   * Find the path from start to goal using A-Star search
   *
   * @param start        The starting location
   * @param goal         The goal location
   * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
   * @return The list of intersections that form the shortest path from
   * start to goal (including both start and goal).
   */
  public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                           GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
    // TODO: Implement this method in WEEK 3

    // Hook for visualization.  See writeup.
    //nodeSearched.accept(next.getLocation());

    return null;
  }


  public static void main(String[] args) {
    System.out.print("Making a new map...");
    MapGraphBackup theMap = new MapGraphBackup();
    System.out.print("DONE. \nLoading the map...");
//    GraphLoader.loadRoadMap("/Users/mateusz/IdeaProjects/UCSDGraphs/data/testdata/simpletest.map", theMap);
    System.out.println("DONE.");

    System.out.println("Num nodes: " + theMap.getNumVertices());
    System.out.println("Num edges: " + theMap.getNumEdges());

//    theMap.printVertices();
//      theMap.printNeighbours();

    List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0, 1.0),
      new GeographicPoint(8.0, -1.0));

    System.out.println(route);

    // You can use this method for testing.

		/* Use this code in Week 3 End of Week Quiz
    MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

  }

}
