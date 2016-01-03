package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * Class to represent a graph node
 */
public class Vertex {


  private GeographicPoint location;
  private List<Edge> neighbours;

  public Vertex(GeographicPoint location) {
    this.location = location;
    neighbours = new ArrayList<>();
  }

  /**
   * Method used to add edges to node
   * @param edge
   */
  public void addNeighbour(Edge edge) {
    neighbours.add(edge);
  }

  /**
   * Method to retrieve a node's location
   * @return location as GeographicPoint
   */
  public GeographicPoint getLocation() {
    return location;
  }

  /**
   * Method to retrieve the node's neighbours
   * @return neighbours as a list of Edges
   */
  public List<Edge> getNeighbours() {
    return neighbours;
  }

  @Override
  public String toString() {
    return "Vertex{" +
      "location=" + location +
      ", neighbours=" + neighbours +
      '}';
  }
}
