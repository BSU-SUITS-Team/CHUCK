using UnityEngine;

public class GridManager : MonoBehaviour
{
    public enum AxisReference { Right, Up, Forward, Left, Down, Back }

    [Header("Grid")]
    public int baseGridX = 30;
    public int baseGridY = 25;
    public int subdivisions = 4;

    [Header("Map Surface")]
    public Transform mapTransform;
    public Transform gridAreaTransform;
    public AxisReference mapWidthAxis = AxisReference.Right;
    public AxisReference mapHeightAxis = AxisReference.Up;
    public float mapWidth = 0.3f;
    public float mapHeight = 0.25f;
    public Vector2 mapCenterOffset = Vector2.zero;

    [Header("Walk mask (same texture as visual map if possible)")]
    public Texture2D obstacleMap;
    [Range(0f, 1f)] public float walkableBrightnessThreshold = 0.6f;
    public bool invertObstacleMask = false;
    public bool flipObstacleX = false;
    public bool flipObstacleY = false;
    public Vector2 obstacleUvMin = Vector2.zero;
    public Vector2 obstacleUvMax = Vector2.one;
    [Tooltip("0 = strict mask only. Higher values grow walkable into neighbors (can leak into black).")]
    [Range(0, 3)] public int walkablePaddingCells = 0;

    public Node[,] grid;

    public Transform ActiveMapTransform => gridAreaTransform != null ? gridAreaTransform : mapTransform;
    public int GridWidth => baseGridX * subdivisions;
    public int GridHeight => baseGridY * subdivisions;
    public float CellWidth => mapWidth / GridWidth;
    public float CellHeight => mapHeight / GridHeight;
    public Vector3 MapAxisX => GetAxisDirection(mapWidthAxis);
    public Vector3 MapAxisY => GetAxisDirection(mapHeightAxis);
    public Vector3 MapNormal => Vector3.Cross(MapAxisX, MapAxisY).normalized;
    public Vector3 MapPlaneCenter => ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;

    private void Start() => GenerateGrid();

    public void GenerateGrid()
    {
        if (ActiveMapTransform == null || obstacleMap == null)
        {
            Debug.LogWarning("GridManager needs map transform and obstacle map.");
            return;
        }

        grid = new Node[GridWidth, GridHeight];

        float minU = Mathf.Min(obstacleUvMin.x, obstacleUvMax.x);
        float maxU = Mathf.Max(obstacleUvMin.x, obstacleUvMax.x);
        float minV = Mathf.Min(obstacleUvMin.y, obstacleUvMax.y);
        float maxV = Mathf.Max(obstacleUvMin.y, obstacleUvMax.y);

        for (int x = 0; x < GridWidth; x++)
        {
            for (int y = 0; y < GridHeight; y++)
            {
                float tx = (x + 0.5f) / GridWidth;
                float ty = (y + 0.5f) / GridHeight;
                if (flipObstacleX) tx = 1f - tx;
                if (flipObstacleY) ty = 1f - ty;

                float u = Mathf.Lerp(minU, maxU, tx);
                float v = Mathf.Lerp(minV, maxV, ty);
                Color pixel = obstacleMap.GetPixelBilinear(Mathf.Clamp01(u), Mathf.Clamp01(v));

                float brightness = (pixel.r + pixel.g + pixel.b) / 3f;
                bool walkable = brightness >= walkableBrightnessThreshold;
                if (invertObstacleMask) walkable = !walkable;

                grid[x, y] = new Node(x, y, walkable);
            }
        }

        if (walkablePaddingCells > 0)
            ExpandWalkableCardinal(walkablePaddingCells);
    }

    private void ExpandWalkableCardinal(int iterations)
    {
        for (int it = 0; it < iterations; it++)
        {
            bool[,] next = new bool[GridWidth, GridHeight];
            for (int x = 0; x < GridWidth; x++)
            {
                for (int y = 0; y < GridHeight; y++)
                {
                    if (grid[x, y].walkable)
                    {
                        next[x, y] = true;
                        continue;
                    }

                    bool neighborWalkable =
                        (x > 0 && grid[x - 1, y].walkable) ||
                        (x < GridWidth - 1 && grid[x + 1, y].walkable) ||
                        (y > 0 && grid[x, y - 1].walkable) ||
                        (y < GridHeight - 1 && grid[x, y + 1].walkable);

                    next[x, y] = neighborWalkable;
                }
            }

            for (int x = 0; x < GridWidth; x++)
            {
                for (int y = 0; y < GridHeight; y++)
                    grid[x, y].walkable = next[x, y];
            }
        }
    }

    public Vector3 GridToWorld(int x, int y)
    {
        Vector3 center = ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;
        Vector3 bottomLeft = center - MapAxisX * (mapWidth * 0.5f) - MapAxisY * (mapHeight * 0.5f);
        return bottomLeft + MapAxisX * ((x + 0.5f) * CellWidth) + MapAxisY * ((y + 0.5f) * CellHeight);
    }

    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        Vector3 center = ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;
        Vector3 local = worldPos - center;
        int gx = Mathf.Clamp(Mathf.FloorToInt((Vector3.Dot(local, MapAxisX) + mapWidth * 0.5f) / CellWidth), 0, GridWidth - 1);
        int gy = Mathf.Clamp(Mathf.FloorToInt((Vector3.Dot(local, MapAxisY) + mapHeight * 0.5f) / CellHeight), 0, GridHeight - 1);
        return new Vector2Int(gx, gy);
    }

    public bool IsInside(Vector2Int p) => p.x >= 0 && p.y >= 0 && p.x < GridWidth && p.y < GridHeight;
    public bool IsWalkable(Vector2Int p) => grid != null && IsInside(p) && grid[p.x, p.y].walkable;

    public Vector2Int FindNearestWalkable(Vector2Int origin, int maxRadius = 12)
    {
        if (grid == null) return origin;
        origin.x = Mathf.Clamp(origin.x, 0, GridWidth - 1);
        origin.y = Mathf.Clamp(origin.y, 0, GridHeight - 1);
        if (IsWalkable(origin)) return origin;

        for (int r = 1; r <= maxRadius; r++)
        {
            int minX = Mathf.Max(0, origin.x - r);
            int maxX = Mathf.Min(GridWidth - 1, origin.x + r);
            int minY = Mathf.Max(0, origin.y - r);
            int maxY = Mathf.Min(GridHeight - 1, origin.y + r);
            for (int x = minX; x <= maxX; x++)
            {
                Vector2Int a = new Vector2Int(x, minY);
                Vector2Int b = new Vector2Int(x, maxY);
                if (IsWalkable(a)) return a;
                if (IsWalkable(b)) return b;
            }
            for (int y = minY + 1; y < maxY; y++)
            {
                Vector2Int a = new Vector2Int(minX, y);
                Vector2Int b = new Vector2Int(maxX, y);
                if (IsWalkable(a)) return a;
                if (IsWalkable(b)) return b;
            }
        }
        return origin;
    }

    private Vector3 GetAxisDirection(AxisReference axis)
    {
        Transform t = ActiveMapTransform;
        if (t == null) return Vector3.right;
        return axis switch
        {
            AxisReference.Right => t.right,
            AxisReference.Up => t.up,
            AxisReference.Forward => t.forward,
            AxisReference.Left => -t.right,
            AxisReference.Down => -t.up,
            AxisReference.Back => -t.forward,
            _ => t.right
        };
    }
}
