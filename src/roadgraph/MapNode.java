package roadgraph;

import geography.GeographicPoint;

public class MapNode implements Comparable
{
	private GeographicPoint vertex;
	private double distance;

	public MapNode (GeographicPoint vertex, double initialDistance)
	{
		this.vertex = vertex;
		distance = initialDistance;
	}

	public GeographicPoint getVertex()
	{
		return vertex;
	}

	public double getDistance()
	{
		return distance;
	}

	public int compareTo(MapNode m)
	{
		if (this.getDistance() > m.getDistance())
		{
			return 1;
		}
		else if (this.getDistance() < m.getDistance())
		{
			return -1;
		}
		else
		{
			return 0;
		}
	}

}