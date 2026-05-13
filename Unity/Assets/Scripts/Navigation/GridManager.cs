using System;
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
    [Tooltip("If on and the map is a RectTransform, grid cells use the quad’s world corners (matches the image). Off = use mapWidth/mapHeight + axes below.")]
    public bool useRectTransformWorldCorners = true;
    [Tooltip("Rect corner layout only: mirror along bottom edge (left↔right on the map).")]
    public bool mirrorRectCornerU = false;
    [Tooltip("Rect corner layout only: mirror along left edge (bottom↔top on the map).")]
    public bool mirrorRectCornerV = false;
    public AxisReference mapWidthAxis = AxisReference.Right;
    public AxisReference mapHeightAxis = AxisReference.Up;
    public float mapWidth = 0.3f;
    public float mapHeight = 0.25f;
    public Vector2 mapCenterOffset = Vector2.zero;

    [Header("Walk mask — can differ from the on-screen map image")]
    [Tooltip("Grayscale/B&W texture used only for walkability sampling (see walkableBrightnessThreshold). The chart image the user sees should be the RawImage/Image material on this map transform — not required to use this texture. Same UV/world mapping as the visual when laid out with useRectTransformWorldCorners.")]
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

    private readonly Vector3[] _rectWorldCorners = new Vector3[4];

    public Transform ActiveMapTransform => gridAreaTransform != null ? gridAreaTransform : mapTransform;
    private bool UsesRectCornerLayout =>
        useRectTransformWorldCorners && ActiveMapTransform is RectTransform;
    public int GridWidth => baseGridX * subdivisions;
    public int GridHeight => baseGridY * subdivisions;
    public float CellWidth => mapWidth / GridWidth;
    public float CellHeight => mapHeight / GridHeight;
    public Vector3 MapAxisX => GetAxisDirection(mapWidthAxis);
    public Vector3 MapAxisY => GetAxisDirection(mapHeightAxis);
    public Vector3 MapNormal => Vector3.Cross(MapAxisX, MapAxisY).normalized;
    /// <summary>RectTransform / panel forward — perpendicular to the map image. Use for small lifts so dots stay on the visible quad.</summary>
    public Vector3 MapFaceOut =>
        ActiveMapTransform != null ? ActiveMapTransform.forward.normalized : Vector3.forward;
    public Vector3 MapPlaneCenter => ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;

    /// <summary>
    /// Plane normal for <see cref="GridToWorld"/> cell layout. Must match for floor world-arrow flattening; axis-only
    /// <see cref="MapNormal"/> diverges when <see cref="useRectTransformWorldCorners"/> is on.
    /// </summary>
    public Vector3 GetGridPlaneNormal()
    {
        if (UsesRectCornerLayout)
        {
            RefreshRectWorldCorners();
            Vector3 c0 = _rectWorldCorners[0];
            Vector3 n = Vector3.Cross(_rectWorldCorners[3] - c0, _rectWorldCorners[1] - c0).normalized;
            if (Vector3.Dot(n, MapFaceOut) < 0f)
                n = -n;
            return n;
        }

        return MapNormal;
    }

    /// <summary>
    /// Path math uses MapAxisX/Y; on world-space UI those can diverge slightly from the drawn quad so cells sit on
    /// different sides of <see cref="MapFaceOut"/>. This projects a world point onto the face plane (pivot + forward).
    /// </summary>
    public Vector3 SnapOntoVisualMapFace(Vector3 worldPoint)
    {
        if (ActiveMapTransform == null)
            return worldPoint;

        if (UsesRectCornerLayout)
        {
            Vector3 n = GetGridPlaneNormal();
            Vector3 c0 = _rectWorldCorners[0];
            return worldPoint - Vector3.Dot(worldPoint - c0, n) * n;
        }

        Vector3 n2 = MapFaceOut;
        Vector3 p0 = ActiveMapTransform.position;
        return worldPoint - Vector3.Dot(worldPoint - p0, n2) * n2;
    }

    private void RefreshRectWorldCorners()
    {
        var rt = ActiveMapTransform as RectTransform;
        rt.GetWorldCorners(_rectWorldCorners);
    }

    /// <summary>0–1 UV on the map image used for walk mask and (with rect corners) world placement — same transforms in both.</summary>
    private void CellToNormalizedMapUV(int x, int y, out float u, out float v)
    {
        u = (x + 0.5f) / GridWidth;
        v = (y + 0.5f) / GridHeight;
        if (UsesRectCornerLayout && mirrorRectCornerU) u = 1f - u;
        if (UsesRectCornerLayout && mirrorRectCornerV) v = 1f - v;
        if (flipObstacleX) u = 1f - u;
        if (flipObstacleY) v = 1f - v;
    }

    /// <summary>Inverse of <see cref="CellToNormalizedMapUV"/> (each step is x → 1−x).</summary>
    private void NormalizedMapUVToCellFraction(float uMap, float vMap, out float uCellFrac, out float vCellFrac)
    {
        float u = uMap;
        float v = vMap;
        if (flipObstacleX) u = 1f - u;
        if (flipObstacleY) v = 1f - v;
        if (UsesRectCornerLayout && mirrorRectCornerU) u = 1f - u;
        if (UsesRectCornerLayout && mirrorRectCornerV) v = 1f - v;
        uCellFrac = u;
        vCellFrac = v;
    }

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
                CellToNormalizedMapUV(x, y, out float tx, out float ty);

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

    /// <summary>Same UV math as <see cref="GridToWorld"/> but using frozen corner positions (e.g. captured at repath) so world visuals do not drift when the map panel moves.</summary>
    public Vector3 GridToWorldUsingRectCorners(int x, int y, Vector3 c0, Vector3 c1, Vector3 c3)
    {
        CellToNormalizedMapUV(x, y, out float u, out float v);
        return c0 + u * (c3 - c0) + v * (c1 - c0);
    }

    /// <summary>Copies the quad’s current world corners (same order as RectTransform.GetWorldCorners). Returns false if not using rect corner layout.</summary>
    public bool TryCopyRectWorldCorners(Vector3[] destFour)
    {
        if (!UsesRectCornerLayout || destFour == null || destFour.Length < 4)
            return false;
        RefreshRectWorldCorners();
        Array.Copy(_rectWorldCorners, destFour, 4);
        return true;
    }

    public Vector3 GridToWorld(int x, int y)
    {
        if (UsesRectCornerLayout)
        {
            RefreshRectWorldCorners();
            Vector3 c0 = _rectWorldCorners[0];
            Vector3 c1 = _rectWorldCorners[1];
            Vector3 c3 = _rectWorldCorners[3];
            return GridToWorldUsingRectCorners(x, y, c0, c1, c3);
        }

        Vector3 center = ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;
        Vector3 bottomLeft = center - MapAxisX * (mapWidth * 0.5f) - MapAxisY * (mapHeight * 0.5f);
        return bottomLeft + MapAxisX * ((x + 0.5f) * CellWidth) + MapAxisY * ((y + 0.5f) * CellHeight);
    }

    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        if (UsesRectCornerLayout)
        {
            RefreshRectWorldCorners();
            Vector3 c0 = _rectWorldCorners[0];
            Vector3 ex = _rectWorldCorners[3] - c0;
            Vector3 ey = _rectWorldCorners[1] - c0;
            return WorldToGridFromRectAxes(worldPos, c0, ex, ey);
        }

        Vector3 center = ActiveMapTransform.position + MapAxisX * mapCenterOffset.x + MapAxisY * mapCenterOffset.y;
        Vector3 local = worldPos - center;
        int gx2 = Mathf.Clamp(Mathf.FloorToInt((Vector3.Dot(local, MapAxisX) + mapWidth * 0.5f) / CellWidth), 0, GridWidth - 1);
        int gy2 = Mathf.Clamp(Mathf.FloorToInt((Vector3.Dot(local, MapAxisY) + mapHeight * 0.5f) / CellHeight), 0, GridHeight - 1);
        return new Vector2Int(gx2, gy2);
    }

    /// <summary>
    /// Same cell mapping as rect-corner <see cref="WorldToGrid"/> but using explicit world corners (e.g. snapshot from PathTest
    /// while the live map transform is disabled after closing the Navigation panel).
    /// </summary>
    public Vector2Int WorldToGridFromCorners(Vector3 worldPos, Vector3 c0, Vector3 c1, Vector3 c3)
    {
        Vector3 ex = c3 - c0;
        Vector3 ey = c1 - c0;
        return WorldToGridFromRectAxes(worldPos, c0, ex, ey);
    }

    private Vector2Int WorldToGridFromRectAxes(Vector3 worldPos, Vector3 c0, Vector3 ex, Vector3 ey)
    {
        Vector3 w = worldPos - c0;
        float a = Vector3.Dot(ex, ex);
        float b = Vector3.Dot(ex, ey);
        float c = Vector3.Dot(ey, ey);
        float d = Vector3.Dot(ex, w);
        float e = Vector3.Dot(ey, w);
        float det = a * c - b * b;
        if (Mathf.Abs(det) < 1e-14f)
            return Vector2Int.zero;
        float u = (c * d - b * e) / det;
        float v = (a * e - b * d) / det;
        u = Mathf.Clamp01(u);
        v = Mathf.Clamp01(v);
        NormalizedMapUVToCellFraction(u, v, out float uCellFrac, out float vCellFrac);
        int gx = Mathf.Clamp(Mathf.Min(GridWidth - 1, Mathf.FloorToInt(uCellFrac * GridWidth)), 0, GridWidth - 1);
        int gy = Mathf.Clamp(Mathf.Min(GridHeight - 1, Mathf.FloorToInt(vCellFrac * GridHeight)), 0, GridHeight - 1);
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
