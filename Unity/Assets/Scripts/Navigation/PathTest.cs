using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

/// <summary>
/// Which local axis of the NavArrow mesh should follow the path tangent. Most FBX arrows use +Y as the tip axis; Unity LookRotation uses +Z.
/// </summary>
public enum WorldArrowMeshForwardAxis
{
    [Tooltip("Align mesh local +Z with path (Quaternion.LookRotation). Default Unity convention.")]
    MeshPlusZAlongPath,
    [Tooltip("Align mesh local +Y with path (Quaternion.FromToRotation). Use when the arrow tip points along +Y in the prefab (common for imported arrows) and would otherwise aim into the floor with +Z mode.")]
    MeshPlusYAlongPath,
    [Tooltip("Align local -Z with path (opposite to +Z). Use if the model’s forward is backward.")]
    MeshMinusZAlongPath
}

public class PathTest : MonoBehaviour
{
    /// <summary>First PathTest in the scene (for UI that cannot reference this component directly).</summary>
    public static PathTest Instance { get; private set; }

    [Header("References")]
    public GridManager grid;
    public Pathfinder pathfinder;
    public Transform player;
    public Transform mapStartPoint;
    public Transform endPoint;
    public TextMeshProUGUI distanceTMP;
    [Tooltip("HoloLens / XR: assign the active scene camera (e.g. Main Camera on the rig). Player builds often lack a reliable Camera.main — without this, Screen Space Camera / some World Space canvases won’t render.")]
    public Camera distanceUICamera;
    [Tooltip("Whole navigation panel / rig to lift so it sits above the floor (local Y offset, applied once on enable).")]
    public Transform navigationVisualRoot;
    [Tooltip("Meters added along local +Y of navigationVisualRoot.")]
    public float navigationMenuRaiseMeters = 0.08f;

    [Header("Map coordinates (feet on chart)")]
    public Vector2 mapCoordMin = new Vector2(-5758f, -10076f);
    public Vector2 mapCoordMax = new Vector2(-5545f, -9940f);
    public Vector2 startCoordinate = new Vector2(-5668f, -10060f);
    public Vector2 endCoordinate = new Vector2(-5635f, -9975f);
    public Vector2 coordinateOffsetFeet = Vector2.zero;
    public Vector2 coordinateScale = Vector2.one;
    public Vector2 coordinateScalePivot = Vector2.zero;
    public bool invertCoordinateX = false;
    public bool invertCoordinateY = false;

    [Header("Movement & repath")]
    public float feetPerMeter = 3.28084f;
    [Tooltip("Path distance is summed in map chart space (same units as mapCoordMin/Max and start/end coordinates). Use 1 if those coords are already feet. If 10 chart units = 10 ft, use 1; if 1 chart unit = 1 ft, use 1.")]
    public float chartDistanceUnitsToFeet = 1f;
    public float repathIntervalSeconds = 0.4f;
    public bool regenerateGridEachRepath = true;
    public int nearestWalkableSearchRadius = 20;

    [Header("Visualization — line")]
    public LineRenderer pathLine;
    public float lineHeightOffset = 0.01f;
    public bool drawDirectLineWhenNoPath = true;

    [Header("Visualization — map arrows (on map surface)")]
    [FormerlySerializedAs("arrowPrefab")]
    public GameObject mapArrowPrefab;
    [Tooltip("Parent for map arrows. At play, children are under a child object named ‘MapPathArrows (runtime)’ so they’re easy to find in the hierarchy.")]
    public Transform mapArrowParent;
    public int mapArrowEveryNNodes = 4;
    [FormerlySerializedAs("arrowHeightOffset")]
    public float mapArrowHeightOffset = 0.03f;
    [Range(-180f, 180f)]
    [Tooltip("Spin around the map normal after aiming (+/-180 flips tip along the surface). Applied after LookRotation; combined order is roll × aim.")]
    public float mapArrowRollAroundNormalDegrees = 0f;

    [Header("Visualization — world follow arrows (floor copy of map path)")]
    public GameObject worldArrowPrefab;
    [Tooltip("Leave empty for world root. Or assign XR Origin / floor content.")]
    public Transform worldArrowParent;
    [Tooltip("If true, worldPathUniformSpaceScale also multiplies arrow mesh size. Leave false to tune path spread and arrow size separately (recommended).")]
    public bool worldArrowMeshScalesWithPath = false;
    [Tooltip("Unused at runtime: world arrows always use map-plane tilt + flatten at the player’s feet (the old “legacy XZ” mode dropped Y and bunched arrows when the path ran along world Y). Kept for serialization.")]
    public bool worldPathMatchMapPlane = true;
    [Tooltip("Extra yaw (degrees, world up) after laying the path flat. For a horizontal path, 180° is equivalent to negating the floor offset from the anchor.")]
    public float worldPathYawOffsetDegrees = 0f;
    [Tooltip("After A*, reverse the node list. Use only if the route still runs goal→start (same geometry, wrong direction along it).")]
    public bool invertPathVisualizationOrder = false;
    [Tooltip("Scales floor path layout only (distance between arrows and overall route size vs the map). Does not change arrow mesh size unless worldArrowMeshScalesWithPath is on.")]
    public float worldPathUniformSpaceScale = 2f;
    [Tooltip("Extra meters along world up after layout (small lift off floor).")]
    public float worldArrowHeightAbovePlayer = 0.02f;
    [Tooltip("World arrow mesh scale only (when worldArrowMeshScalesWithPath is off). Use this for arrow thickness/length; use worldPathUniformSpaceScale for how far apart they sit.")]
    public Vector3 worldArrowScaleMultiplier = new Vector3(2f, 2f, 2f);
    [Min(1)]
    [Tooltip("Place one world arrow every N path segments (grid steps). The map is often highly subdivided, so 1 stacks many arrows in a small area; try 4–8 to match mapArrowEveryNNodes.")]
    public int worldArrowEveryPathSteps = 1;
    public WorldArrowMeshForwardAxis worldArrowMeshForwardAxis = WorldArrowMeshForwardAxis.MeshPlusZAlongPath;
    [Range(-180f, 180f)]
    [Tooltip("Extra local Euler X after base aim (tilt). Try ±90 if Mesh Forward Axis doesn’t match your mesh.")]
    public float worldArrowMeshPitchDegrees = 0f;
    [Range(-180f, 180f)]
    [Tooltip("Yaw after LookRotation (local), in the same Euler as pitch.")]
    public float worldArrowLookYawOffsetDegrees = 0f;
    [Range(-180f, 180f)]
    [Tooltip("Applied first: rotation around world up. Try 180 if headings are mirrored.")]
    public float worldArrowCompassYawDegrees = 0f;
    [Tooltip("Reverses the segment tangent (same positions/spacing). Does not rotate the XR rig.")]
    public bool flipWorldPathDirection = false;
    [Tooltip("When a path exists, snap the player transform’s Y rotation so +Z faces the first floor segment (toward the next path node).")]
    public bool snapPlayerYawToWorldPath = true;

    [Header("Markers & map lift")]
    [Tooltip("Meters along the map RectTransform forward (out of the image). Same for green, red, line, arrows. 0 = on the quad.")]
    public float markerHeightOffset = 0f;
    [Tooltip("Negates the forward lift.")]
    public bool invertSurfaceOffsetDirection = false;

    private readonly List<GameObject> activeMapArrows = new List<GameObject>();
    private readonly List<GameObject> activeWorldArrows = new List<GameObject>();
    private readonly List<Node> currentPath = new List<Node>();

    private Transform mapArrowsRuntimeRoot;

    /// <summary>Latest walkable-snapped cells; line uses these so endpoints match green/red markers.</summary>
    private Vector2Int _lineResolvedStart;
    private Vector2Int _lineResolvedEnd;

    private Vector3 lastPlayerWorldPos;
    private Vector2 currentCoordinate;
    private float repathTimer;
    private Vector3 navigationRootInitialLocalPos;
    private bool navigationRootBaselineStored;
    private bool distanceUiCameraWarningLogged;
    private bool distanceOverlayFixLogged;

    private readonly List<TextMeshProUGUI> boundDistanceReadouts = new List<TextMeshProUGUI>();

    [SerializeField, Tooltip("When false at startup, no path line, arrows, distance, or repaths until SetPathSessionActive(true) (e.g. Find Path in the navigation UI).")]
    private bool pathSessionActive;

    private void Awake()
    {
        if (Instance != null && Instance != this)
            Debug.LogWarning("PathTest: Multiple PathTest components — Instance points to the most recently awakened.");
        Instance = this;
    }

    private void OnDestroy()
    {
        if (Instance == this)
            Instance = null;
    }

    public bool IsPathSessionActive() => pathSessionActive;

    /// <summary>Navigation UI (e.g. sidebar) can bind extra labels updated with the same distance text and visibility as <see cref="distanceTMP"/>.</summary>
    public void BindDistanceReadout(TextMeshProUGUI label)
    {
        if (label == null || boundDistanceReadouts.Contains(label))
            return;
        boundDistanceReadouts.Add(label);
    }

    public void UnbindDistanceReadout(TextMeshProUGUI label)
    {
        if (label == null)
            return;
        boundDistanceReadouts.Remove(label);
    }

    /// <summary>
    /// Point the pathfinding grid at this Navigation instance's map plane. Required when the panel is placed in front of the user
    /// while PathTest/GridManager live in the scene — otherwise markers and map arrows stay at the old scene/prefab origin.
    /// </summary>
    public void BindActiveNavigationMap(RectTransform mapSurface)
    {
        if (grid == null || mapSurface == null)
            return;

        grid.mapTransform = mapSurface;
        grid.gridAreaTransform = null;
        mapArrowParent = mapSurface;

        if (mapArrowsRuntimeRoot != null && mapArrowsRuntimeRoot.parent != mapSurface)
        {
            Destroy(mapArrowsRuntimeRoot.gameObject);
            mapArrowsRuntimeRoot = null;
        }

        grid.GenerateGrid();

        if (pathSessionActive)
            ForceRepath();
    }

    /// <summary>Turns pathfinding visuals and periodic repath on or off (map/world arrows, line, distance, markers).</summary>
    public void SetPathSessionActive(bool active)
    {
        if (pathSessionActive == active)
            return;
        pathSessionActive = active;
        repathTimer = 0f;
        if (!active)
            ClearPathVisualization();
        else
        {
            if (mapStartPoint != null)
                mapStartPoint.gameObject.SetActive(true);
            if (endPoint != null)
                endPoint.gameObject.SetActive(true);
            ForceRepath();
        }
    }

    private void ClearPathVisualization()
    {
        currentPath.Clear();
        DestroyArrowList(activeMapArrows);
        DestroyArrowList(activeWorldArrows);
        if (pathLine != null)
        {
            pathLine.positionCount = 0;
            pathLine.enabled = false;
        }
        foreach (TextMeshProUGUI tmp in EnumerateDistanceReadouts())
        {
            tmp.text = string.Empty;
            tmp.enabled = false;
        }

        if (mapStartPoint != null)
            mapStartPoint.gameObject.SetActive(false);
        if (endPoint != null)
            endPoint.gameObject.SetActive(false);
    }

    private void OnEnable()
    {
        if (navigationVisualRoot == null)
            return;
        if (!navigationRootBaselineStored)
        {
            navigationRootInitialLocalPos = navigationVisualRoot.localPosition;
            navigationRootBaselineStored = true;
        }
        navigationVisualRoot.localPosition = navigationRootInitialLocalPos + Vector3.up * navigationMenuRaiseMeters;
    }

    private void OnDisable()
    {
        if (navigationVisualRoot != null && navigationRootBaselineStored)
            navigationVisualRoot.localPosition = navigationRootInitialLocalPos;
    }

    private void Start()
    {
        if (grid == null || pathfinder == null || player == null)
        {
            Debug.LogWarning("PathTest is missing required references.");
            enabled = false;
            return;
        }

        currentCoordinate = startCoordinate;
        lastPlayerWorldPos = player.position;

        if (grid.grid == null)
            grid.GenerateGrid();

        if (pathLine != null)
            pathLine.useWorldSpace = true;

        if (!pathSessionActive)
            ClearPathVisualization();
    }

    private void Update()
    {
        UpdateCoordinateFromMovement();

        if (!pathSessionActive)
            return;

        repathTimer += Time.deltaTime;
        if (repathTimer >= repathIntervalSeconds)
        {
            repathTimer = 0f;
            ForceRepath();
        }

        UpdateDistanceUI();
    }

    private void LateUpdate()
    {
        if (!pathSessionActive || grid == null || grid.grid == null)
            return;
        if (mapStartPoint == null && endPoint == null)
            return;

        ResolveStartEndGrid(out Vector2Int rs, out Vector2Int re);
        PlaceEndpointMarkers(rs, re);
    }

    private void UpdateCoordinateFromMovement()
    {
        Vector3 delta = player.position - lastPlayerWorldPos;
        lastPlayerWorldPos = player.position;
        if (delta.sqrMagnitude <= Mathf.Epsilon)
            return;

        currentCoordinate.x += delta.x * feetPerMeter;
        currentCoordinate.y += delta.z * feetPerMeter;
    }

    private void ResolveStartEndGrid(out Vector2Int resolvedStart, out Vector2Int resolvedEnd)
    {
        Vector3 startWorld = CoordinateToWorld(currentCoordinate);
        Vector3 endWorld = CoordinateToWorld(endCoordinate);
        Vector2Int startGrid = grid.WorldToGrid(startWorld);
        Vector2Int endGrid = grid.WorldToGrid(endWorld);
        resolvedStart = grid.FindNearestWalkable(startGrid, nearestWalkableSearchRadius);
        resolvedEnd = grid.FindNearestWalkable(endGrid, nearestWalkableSearchRadius);
        _lineResolvedStart = resolvedStart;
        _lineResolvedEnd = resolvedEnd;
    }

    private void PlaceEndpointMarkers(Vector2Int resolvedStart, Vector2Int resolvedEnd)
    {
        Vector3 lift = MapFaceLift(markerHeightOffset);

        if (mapStartPoint != null)
        {
            Vector3 p = grid.SnapOntoVisualMapFace(grid.GridToWorld(resolvedStart.x, resolvedStart.y));
            mapStartPoint.position = p + lift;
        }

        if (endPoint != null)
        {
            Vector3 p = grid.SnapOntoVisualMapFace(grid.GridToWorld(resolvedEnd.x, resolvedEnd.y));
            endPoint.position = p + lift;
        }
    }

    private void ForceRepath()
    {
        if (regenerateGridEachRepath || grid.grid == null)
            grid.GenerateGrid();

        if (grid.grid == null)
            return;

        ResolveStartEndGrid(out Vector2Int resolvedStart, out Vector2Int resolvedEnd);
        PlaceEndpointMarkers(resolvedStart, resolvedEnd);

        List<Node> foundPath = pathfinder.FindPath(resolvedStart, resolvedEnd);

        currentPath.Clear();
        if (foundPath != null)
        {
            EnsurePathRunsStartToEnd(foundPath, resolvedStart, resolvedEnd);
            if (invertPathVisualizationOrder)
                foundPath.Reverse();

            currentPath.AddRange(foundPath);
        }

        DrawPathLine();
        RebuildMapArrows();
        RebuildWorldArrows();
    }

    private Vector3 CoordinateToWorld(Vector2 coordinate)
    {
        Vector2 scaled = coordinateScalePivot + Vector2.Scale(coordinate - coordinateScalePivot, coordinateScale);
        Vector2 adjusted = scaled + coordinateOffsetFeet;
        float tx = Mathf.InverseLerp(mapCoordMin.x, mapCoordMax.x, adjusted.x);
        float ty = Mathf.InverseLerp(mapCoordMin.y, mapCoordMax.y, adjusted.y);

        if (invertCoordinateX) tx = 1f - tx;
        if (invertCoordinateY) ty = 1f - ty;

        tx = Mathf.Clamp01(tx);
        ty = Mathf.Clamp01(ty);

        float x = tx * (grid.GridWidth - 1);
        float y = ty * (grid.GridHeight - 1);
        return grid.GridToWorld(Mathf.RoundToInt(x), Mathf.RoundToInt(y));
    }

    /// <summary>
    /// Inverse of CoordinateToWorld for cell indices: returns position in chart/map coordinate space (before distance scaling).
    /// </summary>
    private Vector2 GridCellToChartCoordinate(int gx, int gy)
    {
        float denomX = Mathf.Max(1, grid.GridWidth - 1);
        float denomY = Mathf.Max(1, grid.GridHeight - 1);
        float tx = gx / denomX;
        float ty = gy / denomY;
        if (invertCoordinateX) tx = 1f - tx;
        if (invertCoordinateY) ty = 1f - ty;

        float adjustedX = Mathf.Lerp(mapCoordMin.x, mapCoordMax.x, tx);
        float adjustedY = Mathf.Lerp(mapCoordMin.y, mapCoordMax.y, ty);
        Vector2 scaled = new Vector2(adjustedX, adjustedY) - coordinateOffsetFeet;
        Vector2 invScale = new Vector2(
            Mathf.Abs(coordinateScale.x) > 1e-8f ? 1f / coordinateScale.x : 0f,
            Mathf.Abs(coordinateScale.y) > 1e-8f ? 1f / coordinateScale.y : 0f);
        return coordinateScalePivot + Vector2.Scale(scaled - coordinateScalePivot, invScale);
    }

    private Vector3 MapFaceLift(float meters)
    {
        if (grid == null || Mathf.Abs(meters) < 1e-8f)
            return Vector3.zero;
        float s = invertSurfaceOffsetDirection ? -1f : 1f;
        return grid.MapFaceOut * meters * s;
    }

    private void DrawPathLine()
    {
        if (pathLine == null || grid == null || grid.grid == null)
            return;

        pathLine.useWorldSpace = true;

        Vector3 lineLift = MapFaceLift(lineHeightOffset);
        Vector3 startBase = grid.SnapOntoVisualMapFace(grid.GridToWorld(_lineResolvedStart.x, _lineResolvedStart.y));
        Vector3 endBase = grid.SnapOntoVisualMapFace(grid.GridToWorld(_lineResolvedEnd.x, _lineResolvedEnd.y));
        Vector3 start = startBase + lineLift;
        Vector3 end = endBase + lineLift;

        if (currentPath.Count == 0)
        {
            if (drawDirectLineWhenNoPath)
            {
                pathLine.positionCount = 2;
                pathLine.SetPosition(0, start);
                pathLine.SetPosition(1, end);
            }
            else
            {
                pathLine.positionCount = 0;
            }
            return;
        }

        pathLine.positionCount = currentPath.Count + 1;
        pathLine.SetPosition(0, start);
        for (int i = 0; i < currentPath.Count; i++)
        {
            Node n = currentPath[i];
            Vector3 cell = grid.SnapOntoVisualMapFace(grid.GridToWorld(n.x, n.y));
            pathLine.SetPosition(i + 1, cell + lineLift);
        }
    }

    private Transform EnsureMapArrowsRuntimeRoot()
    {
        Transform attachTo = mapArrowParent != null ? mapArrowParent : transform;
        if (mapArrowsRuntimeRoot == null)
        {
            var rootGo = new GameObject("MapPathArrows (runtime)");
            rootGo.transform.SetParent(attachTo, false);
            rootGo.transform.localPosition = Vector3.zero;
            rootGo.transform.localRotation = Quaternion.identity;
            rootGo.transform.localScale = Vector3.one;
            mapArrowsRuntimeRoot = rootGo.transform;
        }
        else if (mapArrowsRuntimeRoot.parent != attachTo)
        {
            mapArrowsRuntimeRoot.SetParent(attachTo, false);
        }

        return mapArrowsRuntimeRoot;
    }

    /// <summary>World-space UI parents often have tiny lossy scale; without this, map arrows shrink to invisible.</summary>
    private static void PreserveChildLossyScale(Transform child, Vector3 targetLossyScale)
    {
        Transform p = child.parent;
        if (p == null)
            return;
        Vector3 pl = p.lossyScale;
        child.localScale = new Vector3(
            SafeDivScale(targetLossyScale.x, pl.x),
            SafeDivScale(targetLossyScale.y, pl.y),
            SafeDivScale(targetLossyScale.z, pl.z));
    }

    private static float SafeDivScale(float a, float b) =>
        Mathf.Abs(b) > 1e-8f ? a / b : a;

    private void RebuildMapArrows()
    {
        DestroyArrowList(activeMapArrows);

        if (mapArrowPrefab == null || currentPath.Count < 2)
            return;

        Transform parent = EnsureMapArrowsRuntimeRoot();
        int step = Mathf.Max(1, mapArrowEveryNNodes);
        float arrowAmount = mapArrowHeightOffset >= 0f ? mapArrowHeightOffset : lineHeightOffset;
        Vector3 arrowLift = MapFaceLift(arrowAmount);
        Vector3 faceNormal = grid.MapFaceOut;

        for (int i = 0; i < currentPath.Count - 1; i += step)
        {
            Node current = currentPath[i];
            Node next = currentPath[Mathf.Min(i + 1, currentPath.Count - 1)];

            Vector3 currentPos = grid.SnapOntoVisualMapFace(grid.GridToWorld(current.x, current.y));
            Vector3 nextPos = grid.SnapOntoVisualMapFace(grid.GridToWorld(next.x, next.y));
            Vector3 direction = (nextPos - currentPos).normalized;
            if (direction.sqrMagnitude <= Mathf.Epsilon)
                continue;

            Vector3 spawnPos = currentPos + arrowLift;
            Quaternion rotation = Quaternion.AngleAxis(mapArrowRollAroundNormalDegrees, faceNormal)
                * Quaternion.LookRotation(direction, faceNormal);

            GameObject instance = Instantiate(mapArrowPrefab);
            instance.name = $"{mapArrowPrefab.name} (map seg {i})";
            instance.SetActive(true);
            Transform t = instance.transform;
            Vector3 desiredWorldScale = t.lossyScale;
            t.SetPositionAndRotation(spawnPos, rotation);
            t.SetParent(parent, true);
            PreserveChildLossyScale(t, desiredWorldScale);
            activeMapArrows.Add(instance);
        }
    }

    private void RebuildWorldArrows()
    {
        DestroyArrowList(activeWorldArrows);

        if (worldArrowPrefab == null || player == null || grid == null || grid.grid == null || currentPath.Count < 2)
            return;

        float spaceScale = Mathf.Max(0.001f, worldPathUniformSpaceScale);
        Vector3 visualScaleMul = worldArrowMeshScalesWithPath
            ? Vector3.Scale(worldArrowScaleMultiplier, new Vector3(spaceScale, spaceScale, spaceScale))
            : worldArrowScaleMultiplier;

        int pathStep = Mathf.Max(1, worldArrowEveryPathSteps);
        Quaternion tiltMapPlaneToWorldUp = Quaternion.FromToRotation(grid.GetGridPlaneNormal(), Vector3.up);
        Quaternion yaw = Quaternion.Euler(0f, worldPathYawOffsetDegrees, 0f);
        Vector3 startOnMap = grid.GridToWorld(currentPath[0].x, currentPath[0].y);
        Quaternion playerMeshYawOnly = Quaternion.Euler(0f, worldArrowLookYawOffsetDegrees, 0f);

        if (snapPlayerYawToWorldPath && currentPath.Count >= 2)
        {
            Vector3 p0 = WorldArrowFloorFromMapCell(currentPath[0].x, currentPath[0].y, startOnMap, tiltMapPlaneToWorldUp, yaw, spaceScale);
            Vector3 p1 = WorldArrowFloorFromMapCell(currentPath[1].x, currentPath[1].y, startOnMap, tiltMapPlaneToWorldUp, yaw, spaceScale);
            Vector3 flat = p1 - p0;
            flat.y = 0f;
            if (flat.sqrMagnitude > 1e-8f)
            {
                Quaternion facePath = Quaternion.LookRotation(flat.normalized, Vector3.up) * playerMeshYawOnly;
                Vector3 e = player.eulerAngles;
                e.y = facePath.eulerAngles.y;
                player.eulerAngles = e;
            }
        }

        for (int i = 0; i < currentPath.Count - 1; i += pathStep)
        {
            int nextIdx = Mathf.Min(i + pathStep, currentPath.Count - 1);
            Node current = currentPath[i];
            Node next = currentPath[nextIdx];

            Vector3 worldPos = WorldArrowFloorFromMapCell(current.x, current.y, startOnMap, tiltMapPlaneToWorldUp, yaw, spaceScale);
            Vector3 nextWorld = WorldArrowFloorFromMapCell(next.x, next.y, startOnMap, tiltMapPlaneToWorldUp, yaw, spaceScale);
            Vector3 tangent = nextWorld - worldPos;
            if (flipWorldPathDirection)
                tangent = -tangent;
            if (tangent.sqrMagnitude <= 1e-12f)
                continue;

            Vector3 direction = tangent.normalized;
            Quaternion rotation =
                Quaternion.AngleAxis(worldArrowCompassYawDegrees, Vector3.up)
                * WorldArrowBaseRotation(direction, worldArrowMeshForwardAxis)
                * Quaternion.Euler(worldArrowMeshPitchDegrees, worldArrowLookYawOffsetDegrees, 0f);
            GameObject instance = Instantiate(worldArrowPrefab);
            Transform t = instance.transform;
            t.SetPositionAndRotation(worldPos, rotation);
            t.localScale = Vector3.Scale(t.localScale, visualScaleMul);
            if (worldArrowParent != null)
                t.SetParent(worldArrowParent, true);
            activeWorldArrows.Add(instance);
        }
    }

    /// <summary>
    /// Path start cell currentPath[0]: floor point = player + yaw * flatten(scale * (cellWorld - startCellWorld)).
    /// </summary>
    private Vector3 WorldArrowFloorFromMapCell(int gx, int gy, Vector3 startOnMap, Quaternion tiltToHorizontal, Quaternion yaw, float spaceScale)
    {
        Vector3 deltaOnMap = grid.GridToWorld(gx, gy) - startOnMap;
        Vector3 flattened = Vector3.ProjectOnPlane(tiltToHorizontal * deltaOnMap * spaceScale, Vector3.up);
        return player.position + yaw * flattened + Vector3.up * worldArrowHeightAbovePlayer;
    }

    /// <summary>
    /// A* should return [start … end]. If the list is [end … start] or ends at the start cell while the head is not the start, reverse so index 0 is the journey origin.
    /// </summary>
    private static void EnsurePathRunsStartToEnd(List<Node> path, Vector2Int resolvedStart, Vector2Int resolvedEnd)
    {
        if (path == null || path.Count <= 1)
            return;

        bool sameCell = resolvedStart.x == resolvedEnd.x && resolvedStart.y == resolvedEnd.y;

        Node head = path[0];
        Node tail = path[path.Count - 1];
        bool headIsStart = head.x == resolvedStart.x && head.y == resolvedStart.y;
        bool tailIsEnd = tail.x == resolvedEnd.x && tail.y == resolvedEnd.y;
        if (headIsStart && tailIsEnd)
            return;

        bool headIsEnd = head.x == resolvedEnd.x && head.y == resolvedEnd.y;
        bool tailIsStart = tail.x == resolvedStart.x && tail.y == resolvedStart.y;

        if (!sameCell && headIsEnd && tailIsStart)
        {
            path.Reverse();
            return;
        }

        if (!headIsStart && tailIsStart)
            path.Reverse();
    }

    private static Quaternion WorldArrowBaseRotation(Vector3 direction, WorldArrowMeshForwardAxis axis)
    {
        direction = direction.normalized;
        switch (axis)
        {
            case WorldArrowMeshForwardAxis.MeshPlusYAlongPath:
                if (Mathf.Abs(Vector3.Dot(direction, Vector3.up)) > 0.998f)
                    return Quaternion.LookRotation(direction, Vector3.forward);
                return Quaternion.FromToRotation(Vector3.up, direction);
            case WorldArrowMeshForwardAxis.MeshMinusZAlongPath:
                return Quaternion.LookRotation(-direction, Vector3.up);
            default:
                return Quaternion.LookRotation(direction, Vector3.up);
        }
    }

    private static void DestroyArrowList(List<GameObject> list)
    {
        for (int i = 0; i < list.Count; i++)
        {
            if (list[i] != null)
                Destroy(list[i]);
        }
        list.Clear();
    }

    private void UpdateDistanceUI()
    {
        PruneDestroyedDistanceReadouts();

        if (!HasAnyDistanceReadout())
            return;

        if (!pathSessionActive)
        {
            foreach (TextMeshProUGUI tmp in EnumerateDistanceReadouts())
            {
                tmp.enabled = false;
                tmp.text = string.Empty;
            }

            return;
        }

        string line = $"Distance: {GetPathDistanceFeet():F1} ft";
        foreach (TextMeshProUGUI tmp in EnumerateDistanceReadouts())
        {
            tmp.enabled = true;
            tmp.text = line;
            EnsureDistanceLabelRenderable(tmp);
            if (!tmp.gameObject.activeInHierarchy)
                tmp.gameObject.SetActive(true);
            tmp.ForceMeshUpdate(true);
        }
    }

    private void PruneDestroyedDistanceReadouts()
    {
        boundDistanceReadouts.RemoveAll(static x => x == null);
    }

    private bool HasAnyDistanceReadout()
    {
        if (distanceTMP != null)
            return true;
        PruneDestroyedDistanceReadouts();
        return boundDistanceReadouts.Count > 0;
    }

    private IEnumerable<TextMeshProUGUI> EnumerateDistanceReadouts()
    {
        if (distanceTMP != null)
            yield return distanceTMP;
        for (int i = 0; i < boundDistanceReadouts.Count; i++)
        {
            if (boundDistanceReadouts[i] != null)
                yield return boundDistanceReadouts[i];
        }
    }

    private void EnsureDistanceLabelRenderable(TextMeshProUGUI tmp)
    {
        Canvas canvas = tmp.canvas;
        if (canvas == null)
            return;

        canvas.enabled = true;
        if (!canvas.gameObject.activeInHierarchy)
            canvas.gameObject.SetActive(true);

        Camera cam = ResolveDistanceCanvasCamera();

        if (canvas.renderMode == RenderMode.ScreenSpaceOverlay && cam != null)
        {
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = cam;
            if (canvas.planeDistance < 0.01f || canvas.planeDistance > 500f)
                canvas.planeDistance = 0.5f;
            if (!distanceOverlayFixLogged)
            {
                distanceOverlayFixLogged = true;
                Debug.Log(
                    "PathTest: Distance label canvas was Screen Space Overlay — switched to Screen Space Camera for XR/HoloLens. Prefer binding distance to the Navigation panel TextMeshPro.");
            }
        }
        else if (canvas.renderMode == RenderMode.WorldSpace || canvas.renderMode == RenderMode.ScreenSpaceCamera)
        {
            if (cam != null && canvas.worldCamera != cam)
                canvas.worldCamera = cam;
            else if (cam == null && !distanceUiCameraWarningLogged)
            {
                distanceUiCameraWarningLogged = true;
                Debug.LogWarning(
                    "PathTest: No camera found for distance UI canvas. Assign PathTest.distanceUICamera to your HoloLens/XR rig camera so the distance text renders in player builds.");
            }
        }

        RectTransform crt = canvas.transform as RectTransform;
        if (crt != null && crt.localScale.sqrMagnitude < 1e-8f)
            crt.localScale = Vector3.one;

        if (tmp.rectTransform.localScale.sqrMagnitude < 1e-8f)
            tmp.rectTransform.localScale = Vector3.one;
    }

    private Camera ResolveDistanceCanvasCamera()
    {
        if (distanceUICamera != null && distanceUICamera.isActiveAndEnabled)
            return distanceUICamera;
        if (Camera.main != null && Camera.main.isActiveAndEnabled)
            return Camera.main;
        Camera[] cams = FindObjectsOfType<Camera>();
        for (int i = 0; i < cams.Length; i++)
        {
            if (cams[i] != null && cams[i].isActiveAndEnabled && cams[i].gameObject.activeInHierarchy)
                return cams[i];
        }
        return null;
    }

    private float GetPathDistanceFeet()
    {
        float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
        if (currentPath.Count == 0)
            return Vector2.Distance(currentCoordinate, endCoordinate) * toFeet;

        float totalChart = 0f;
        Vector2 previous = currentCoordinate;
        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector2 current = GridCellToChartCoordinate(currentPath[i].x, currentPath[i].y);
            totalChart += Vector2.Distance(previous, current);
            previous = current;
        }
        return totalChart * toFeet;
    }
}
