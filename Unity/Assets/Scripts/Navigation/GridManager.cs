using UnityEngine;

public class GridManager : MonoBehaviour
{
    [Header("Grid Settings")]
    public int baseGridX = 30;
    public int baseGridY = 25;
    public int subdivisions = 4;

    [Header("Map (Plane)")]
    public Transform mapTransform;
    public float mapWidth = 0.3f;
    public float mapHeight = 0.25f;

    [Header("Obstacle Mask")]
    public Texture2D obstacleMap;

    public Node[,] grid;

    public int GridWidth => baseGridX * subdivisions;
    public int GridHeight => baseGridY * subdivisions;

    public float CellWidth => mapWidth / GridWidth;
    public float CellHeight => mapHeight / GridHeight;

    void Start()
    {
        GenerateGrid();
    }

    void GenerateGrid()
    {
        grid = new Node[GridWidth, GridHeight];

        int texWidth = obstacleMap.width;
        int texHeight = obstacleMap.height;

        for (int x = 0; x < GridWidth; x++)
        {
            for (int y = 0; y < GridHeight; y++)
            {
                // Direct 1:1 mapping from grid to texture
                int px = Mathf.FloorToInt((float)x / GridWidth * texWidth);
                int py = Mathf.FloorToInt((float)y / GridHeight * texHeight);

                px = Mathf.Clamp(px, 0, texWidth - 1);
                py = Mathf.Clamp(py, 0, texHeight - 1);

                Color pixel = obstacleMap.GetPixel(px, py);

                // White = walkable, anything else = blocked
                bool isBlocked = Physics.CheckBox(
                    GridToWorld(x, y),
                    new Vector3(CellWidth / 2f, 0.01f, CellHeight / 2f)
                );

                grid[x, y] = new Node(x, y, !isBlocked);
            }
        }
    }

    public Vector3 GridToWorld(int x, int y)
    {
        Vector3 bottomLeft =
            mapTransform.position
            - mapTransform.right * (mapWidth / 2f)
            - mapTransform.forward * (mapHeight / 2f);

        return bottomLeft
            + mapTransform.right * ((x + 0.5f) * CellWidth)
            + mapTransform.forward * ((y + 0.5f) * CellHeight);
    }

    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        Vector3 local = worldPos - mapTransform.position;

        float x = Vector3.Dot(local, mapTransform.right) + mapWidth / 2f;
        float y = Vector3.Dot(local, mapTransform.forward) + mapHeight / 2f;

        int gx = Mathf.FloorToInt(x / CellWidth);
        int gy = Mathf.FloorToInt(y / CellHeight);

        gx = Mathf.Clamp(gx, 0, GridWidth - 1);
        gy = Mathf.Clamp(gy, 0, GridHeight - 1);

        return new Vector2Int(gx, gy);
    }

    void OnDrawGizmos()
    {
        if (grid == null || mapTransform == null) return;

        for (int x = 0; x < GridWidth; x++)
        {
            for (int y = 0; y < GridHeight; y++)
            {
                if (!grid[x, y].walkable)
                {
                    Vector3 pos = GridToWorld(x, y) + mapTransform.up * 0.001f;
                    Gizmos.color = Color.red;
                    Gizmos.DrawCube(pos, Vector3.one * 0.002f);
                }
            }
        }
    }
}