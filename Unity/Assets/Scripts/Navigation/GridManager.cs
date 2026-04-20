using UnityEngine;

public class GridManager : MonoBehaviour
{
    public int baseGridX = 30;
    public int baseGridY = 25;

    public int subdivisions = 4; // 4x4 = 16 per cell

    public float mapWidth = 20f;  // meters
    public float mapHeight = 16f;

    public Node[,] grid;

    public int GridWidth => baseGridX * subdivisions;
    public int GridHeight => baseGridY * subdivisions;

    public float CellSize => mapWidth / GridWidth;

    void Start()
    {
        GenerateGrid();
    }

    void GenerateGrid()
    {
        grid = new Node[GridWidth, GridHeight];

        for (int x = 0; x < GridWidth; x++)
        {
            for (int y = 0; y < GridHeight; y++)
            {
                grid[x, y] = new Node(x, y, true);
            }
        }
    }

    void OnDrawGizmos()
    {
        if (grid == null) return;

        Gizmos.color = Color.gray;

        for (int x = 0; x < GridWidth; x++)
        {
            for (int y = 0; y < GridHeight; y++)
            {
                Vector3 worldPos = GridToWorld(x, y);
                Gizmos.DrawWireCube(worldPos, Vector3.one * CellSize);
            }
        }
    }

    Vector3 GridToWorld(int x, int y)
    {
        return new Vector3(
            x * CellSize + CellSize / 2,
            0,
            y * CellSize + CellSize / 2
        );
    }
}
