package roadgraph;

import geography.GeographicPoint;

public class Edge {
	
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	private boolean visited;
	
	public Edge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
	{
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getOrigin()
	{
		return from;
	}
	
	public GeographicPoint getDestination()
	{
		return to;
	}
	
	public String getRoadName()
	{
		return roadName;
	}
	
	public String getRoadType()
	{
		return roadType;
	}
	
	public double getLength()
	{
		return length;
	}
	
	public void setVisited(boolean visited)
	{
		this.visited = visited;
	}
	
	public boolean isVisited()
	{
		return visited;
	}

}
