/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashSet;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.PriorityQueue;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private int numOfVertices;
	private int numOfEdges;
	//Hashmap to store outbound edges of each vertex
	private HashMap<GeographicPoint, List<Edge>> neighbors;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		numOfVertices = 0;
		numOfEdges = 0;
		neighbors = new HashMap<GeographicPoint, List<Edge>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numOfVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return neighbors.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numOfEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (location == null)
		{
			return false;
		}
		
		//create a list object in neighbors if vertex did not already exist
		if (!neighbors.containsKey(location))
		{
			neighbors.put(location, new LinkedList<Edge>());
			numOfVertices++;
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if (from == null || to == null || roadName == null || roadType == null || length < 0)
		{
			throw new IllegalArgumentException("Arguments cannot be null.");
		}
		
		if (!neighbors.containsKey(from) || !neighbors.containsKey(to))
		{
			throw new IllegalArgumentException("Vertices must be added to graph.");
		}
		
		//add outbound edge to the "from" vertex
		List<Edge> edges = neighbors.get(from);
		edges.add(new Edge(from, to, roadName, roadType, length));
		numOfEdges++;
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		//checks if goal was reached
		boolean reachedGoal = false;
		//parent map to keep track of how we got to the vertex
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		//keeps track of vertices already visited
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		queue.add(start);
		
		while (!queue.isEmpty() && !reachedGoal)
		{
			GeographicPoint origin = queue.remove();
			nodeSearched.accept(origin);
			List<Edge> edges = neighbors.get(origin);
			for (Edge e: edges)
			{
				GeographicPoint destination = e.getDestination();
				if (!visited.contains(destination))
				{
					queue.add(destination);
					visited.add(destination);
					parentMap.put(destination, origin);	
				}
				//if the goal is reached, set reachedGoal to true and exit the loop
				if (destination.equals(goal))
				{
					reachedGoal = true;
					break;
				}
			}
		}
		
		//returns null if goal was never reached
		if (!reachedGoal)
		{
			return null;
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return buildPath(start, goal, parentMap);
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distTo = new HashMap<GeographicPoint, Double>();
		boolean reachedGoal = false;
		
		Set<GeographicPoint> vertices = getVertices();

		for (GeographicPoint vertex: vertices)
		{
			distTo.put(vertex, Double.POSITIVE_INFINITY);
		}

		parentMap.put(start, null);
		distTo.put(start, 0.0);
		MapNode startNode = new MapNode(start, 0.0);
		pq.add(startNode);

		while (!pq.isEmpty())
		{
			MapNode mp = pq.poll();
			GeographicPoint currentVertex = mp.getVertex();

			if (!visited.contains(currentVertex))
			{
				//System.out.print("d: ");
				//System.out.println(currentVertex.toString());
				visited.add(currentVertex);
				nodeSearched.accept(currentVertex);
				List<Edge> edgeList = neighbors.get(currentVertex);
				for (Edge e: edgeList)
				{
					GeographicPoint neighborVertex = e.getDestination();
					double distanceFromStart = distTo.get(currentVertex) + e.getLength();

					if (distanceFromStart < distTo.get(neighborVertex))
					{
						distTo.put(neighborVertex, distanceFromStart);
						parentMap.put(neighborVertex, currentVertex);
						MapNode newNode = new MapNode(neighborVertex, distanceFromStart);
						pq.add(newNode);
					}
				}
			}

			if (currentVertex.equals(goal))
			{
				reachedGoal = true;
				break;
			}
			
		}

		if (!reachedGoal)
		{
			return null;
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return buildPath(start, goal, parentMap);	
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distTo = new HashMap<GeographicPoint, Double>();
		HashMap<GeographicPoint, Double> distRemaining = new HashMap<GeographicPoint, Double>();
		boolean reachedGoal = false;

		Set<GeographicPoint> vertices = getVertices();

		for (GeographicPoint vertex: vertices)
		{
			distTo.put(vertex, Double.POSITIVE_INFINITY);
			distRemaining.put(vertex, vertex.distance(goal));
		}

		parentMap.put(start, null);
		distTo.put(start, 0.0);
		MapNode startNode = new MapNode(start, distTo.get(start) + distRemaining.get(start));
		pq.add(startNode);

		while (!pq.isEmpty())
		{
			MapNode mp = pq.poll();
			GeographicPoint currentVertex = mp.getVertex();

			if (!visited.contains(currentVertex))
			{
				//System.out.print("a: ");
				//System.out.println(currentVertex.toString());
				visited.add(currentVertex);
				nodeSearched.accept(currentVertex);
				List<Edge> edgeList = neighbors.get(currentVertex);
				for (Edge e: edgeList)
				{
					GeographicPoint neighborVertex = e.getDestination();
					double distanceFromStart = distTo.get(currentVertex) + e.getLength();
					double predictedTotal = distanceFromStart + distRemaining.get(neighborVertex);

					if (predictedTotal < distTo.get(neighborVertex) + distRemaining.get(neighborVertex))
					{
						distTo.put(neighborVertex, distanceFromStart);
						parentMap.put(neighborVertex, currentVertex);
						MapNode newNode = new MapNode(neighborVertex, predictedTotal);
						pq.add(newNode);
					}
				}
			}

			if (currentVertex.equals(goal))
			{
				reachedGoal = true;
				break;
			}
					
		}

		if (!reachedGoal)
		{
			return null;
		}
				
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return buildPath(start, goal, parentMap);	
	
	}
	

	private List<GeographicPoint> buildPath(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap)
	{
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		//builds path from one vertex after start, to the goal
		for (GeographicPoint p = goal; p != start; p = parentMap.get(p))
		{
			path.add(0, p);
		}
		//add start to the path
		path.add(0, start);

		return path;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		//MapGraph theMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		//System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		
		
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(1, 1);
		GeographicPoint end = new GeographicPoint(8, -1);
		
		/**
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		*/
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		System.out.println("dijkstra");
		for (GeographicPoint g: route)
		{
			System.out.println(g.toString());
		}
		System.out.println();
		System.out.println("aStar");
		for (GeographicPoint g2: route2)
		{
			System.out.println(g2.toString());
		}
		/**
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
