package roadgraph;

import geography.GeographicPoint;

/**
 * Class to represent a graph edge
 */
public class Edge {
  // Two endpoints of the edge
  private GeographicPoint start, destination;
  // Name and type of road that edge represents
  private String streetName, streetType;
  // Length of edge
  private double distance;

  public Edge(GeographicPoint start, GeographicPoint destination, String streetName,
              String streetType, double distance) {
    this.start = start;
    this.destination = destination;
    this.streetName = streetName;
    this.streetType = streetType;
    this.distance = distance;
  }

  /**
   * Accessor method to retrieve start point
   * @return start as GeographicPoint
   */
  public GeographicPoint getStart() {
    return start;
  }

  /**
   * Accessor method to retrieve end point
   * @return destination as GeographicPoint
   */
  public GeographicPoint getDestination() {
    return destination;
  }

  @Override
  public String toString() {
    return "Edge{" +
      "start=" + start +
      ", destination=" + destination +
      ", streetName='" + streetName + '\'' +
      ", streetType='" + streetType + '\'' +
      ", distance=" + distance +
      '}';
  }
}

