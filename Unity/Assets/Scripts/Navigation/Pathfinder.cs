using System.Collections.Generic;
using UnityEngine;

public class Pathfinder : MonoBehaviour
{
    public GridManager gridManager;
    public bool preventDiagonalCornerCutting = true;

    public List<Node> FindPath(Vector2Int start, Vector2Int end)
    {
        if (gridManager == null || gridManager.grid == null)
            return null;

        Node startNode = gridManager.grid[start.x, start.y];
        Node endNode = gridManager.grid[end.x, end.y];

        if (!startNode.walkable || !endNode.walkable)
            return null;

        List<Node> openList = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();

        foreach (Node n in gridManager.grid)
        {
            n.gCost = float.MaxValue;
            n.hCost = 0;
            n.parent = null;
        }

        startNode.gCost = 0;
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            Node current = openList[0];
            foreach (Node n in openList)
            {
                if (n.fCost < current.fCost || (n.fCost == current.fCost && n.hCost < current.hCost))
                    current = n;
            }

            openList.Remove(current);
            closedSet.Add(current);

            if (current == endNode)
                return RetracePath(startNode, endNode);

            foreach (Node neighbor in GetNeighbors(current))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                    continue;

                int dx = Mathf.Abs(current.x - neighbor.x);
                int dy = Mathf.Abs(current.y - neighbor.y);
                float moveCost = (dx == 1 && dy == 1) ? 14f : 10f;
                float newCost = current.gCost + moveCost;

                if (newCost < neighbor.gCost)
                {
                    neighbor.gCost = newCost;
                    neighbor.hCost = 10f * (Mathf.Abs(neighbor.x - endNode.x) + Mathf.Abs(neighbor.y - endNode.y));
                    neighbor.parent = current;
                    if (!openList.Contains(neighbor))
                        openList.Add(neighbor);
                }
            }
        }

        return null;
    }

    private List<Node> RetracePath(Node start, Node end)
    {
        List<Node> path = new List<Node>();
        Node current = end;
        while (true)
        {
            path.Add(current);
            if (current == start)
                break;
            current = current.parent;
        }

        path.Reverse();
        return path;
    }

    private List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0) continue;

                int x = node.x + dx;
                int y = node.y + dy;

                if (x < 0 || y < 0 || x >= gridManager.GridWidth || y >= gridManager.GridHeight)
                    continue;

                if (preventDiagonalCornerCutting && dx != 0 && dy != 0)
                {
                    Node n1 = gridManager.grid[node.x + dx, node.y];
                    Node n2 = gridManager.grid[node.x, node.y + dy];
                    if (!n1.walkable || !n2.walkable)
                        continue;
                }

                neighbors.Add(gridManager.grid[x, y]);
            }
        }

        return neighbors;
    }
}
