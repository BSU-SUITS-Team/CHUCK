using UnityEngine;
using System.Collections.Generic;

public class PathTest : MonoBehaviour
{
    public GridManager grid;
    public Pathfinder pathfinder;

    public Transform startPoint;
    public Transform endPoint;

    List<Node> path;

    void Update()
    {
        HandlePlacement();

        Vector2Int start = grid.WorldToGrid(startPoint.position);
        Vector2Int end = grid.WorldToGrid(endPoint.position);

        //DO NOT modify grid walkability
        if (!grid.grid[start.x, start.y].walkable ||
            !grid.grid[end.x, end.y].walkable)
        {
            path = null;
            return;
        }

        path = pathfinder.FindPath(start, end);

        DrawPath();
    }

    void HandlePlacement()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            if (hit.collider.gameObject == grid.mapTransform.gameObject)
            {
                if (Input.GetMouseButtonDown(0))
                    startPoint.position = hit.point + grid.mapTransform.up * 0.001f;

                if (Input.GetMouseButtonDown(1))
                    endPoint.position = hit.point + grid.mapTransform.up * 0.001f;
            }
        }
    }

    void DrawPath()
    {
        if (path == null) return;

        Vector3 normal = grid.mapTransform.up;

        //handle same-cell case
        if (path.Count == 0)
        {
            Debug.DrawLine(
                startPoint.position + normal * 0.001f,
                endPoint.position + normal * 0.001f,
                Color.cyan
            );
            return;
        }

        for (int i = 0; i < path.Count - 1; i++)
        {
            Vector3 a = grid.GridToWorld(path[i].x, path[i].y);
            Vector3 b = grid.GridToWorld(path[i + 1].x, path[i + 1].y);

            Debug.DrawLine(
                a + normal * 0.001f,
                b + normal * 0.001f,
                Color.cyan
            );
        }
    }
}