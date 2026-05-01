using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;

public class PathTest : MonoBehaviour
{
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
    [Tooltip("Parent for map arrows (e.g. this object or Map). Uses map normal for rotation.")]
    public Transform mapArrowParent;
    public int mapArrowEveryNNodes = 4;
    [FormerlySerializedAs("arrowHeightOffset")]
    public float mapArrowHeightOffset = 0.03f;

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
    [Range(-180f, 180f)]
    [Tooltip("Pitch after LookRotation (local). Default 0 = vertical cylinder on the floor, +Z aimed along the path.")]
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

    [Header("Markers & surface offset")]
    public float markerHeightOffset = 0.0125f;
    public bool invertSurfaceOffsetDirection = false;
    public bool autoFaceUserForOffsets = true;

    private readonly List<GameObject> activeMapArrows = new List<GameObject>();
    private readonly List<GameObject> activeWorldArrows = new List<GameObject>();
    private readonly List<Node> currentPath = new List<Node>();

    private Vector3 lastPlayerWorldPos;
    private Vector2 currentCoordinate;
    private float repathTimer;
    private Vector3 navigationRootInitialLocalPos;
    private bool navigationRootBaselineStored;
    private bool distanceUiCameraWarningLogged;

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

        if (distanceTMP == null)
            Debug.LogWarning("PathTest: distanceTMP is not assigned — distance will not show. Link the Canvas → Distance TextMeshProUGUI.");

        ForceRepath();
    }

    private void Update()
    {
        UpdateCoordinateFromMovement();

        repathTimer += Time.deltaTime;
        if (repathTimer >= repathIntervalSeconds)
        {
            repathTimer = 0f;
            ForceRepath();
        }

        // RebuildWorldArrows(); // TODO: re-enable when world arrows should follow the player along the path each frame.
        UpdateDistanceUI();
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

    private void ForceRepath()
    {
        if (regenerateGridEachRepath || grid.grid == null)
            grid.GenerateGrid();

        if (grid.grid == null)
            return;

        Vector3 startWorld = CoordinateToWorld(currentCoordinate);
        Vector3 endWorld = CoordinateToWorld(endCoordinate);

        Vector2Int startGrid = grid.WorldToGrid(startWorld);
        Vector2Int endGrid = grid.WorldToGrid(endWorld);
        Vector2Int resolvedStart = grid.FindNearestWalkable(startGrid, nearestWalkableSearchRadius);
        Vector2Int resolvedEnd = grid.FindNearestWalkable(endGrid, nearestWalkableSearchRadius);

        if (mapStartPoint != null)
            mapStartPoint.position = grid.GridToWorld(resolvedStart.x, resolvedStart.y) + SurfaceOffset(markerHeightOffset);
        if (endPoint != null)
            endPoint.position = grid.GridToWorld(resolvedEnd.x, resolvedEnd.y) + SurfaceOffset(markerHeightOffset);

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

    private Vector3 SurfaceOffset(float amount)
    {
        Vector3 normal = grid.MapNormal;
        float direction = 1f;

        if (autoFaceUserForOffsets)
        {
            Transform reference = player != null ? player : Camera.main != null ? Camera.main.transform : null;
            if (reference != null && grid.ActiveMapTransform != null)
            {
                Vector3 toReference = (reference.position - grid.ActiveMapTransform.position).normalized;
                if (Vector3.Dot(normal, toReference) < 0f)
                    direction *= -1f;
            }
        }

        if (invertSurfaceOffsetDirection)
            direction *= -1f;

        return normal * amount * direction;
    }

    private void DrawPathLine()
    {
        if (pathLine == null)
            return;

        pathLine.useWorldSpace = true;

        Vector3 start = CoordinateToWorld(currentCoordinate) + SurfaceOffset(lineHeightOffset);
        Vector3 end = CoordinateToWorld(endCoordinate) + SurfaceOffset(lineHeightOffset);

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
            pathLine.SetPosition(i + 1, grid.GridToWorld(n.x, n.y) + SurfaceOffset(lineHeightOffset));
        }
    }

    private void RebuildMapArrows()
    {
        DestroyArrowList(activeMapArrows);

        if (mapArrowPrefab == null || currentPath.Count < 2)
            return;

        Transform parent = mapArrowParent != null ? mapArrowParent : transform;
        int step = Mathf.Max(1, mapArrowEveryNNodes);

        for (int i = 0; i < currentPath.Count - 1; i += step)
        {
            Node current = currentPath[i];
            Node next = currentPath[Mathf.Min(i + 1, currentPath.Count - 1)];

            Vector3 currentPos = grid.GridToWorld(current.x, current.y);
            Vector3 nextPos = grid.GridToWorld(next.x, next.y);
            Vector3 direction = (nextPos - currentPos).normalized;
            if (direction.sqrMagnitude <= Mathf.Epsilon)
                continue;

            Quaternion rotation = Quaternion.LookRotation(direction, grid.MapNormal);
            Vector3 spawnPos = currentPos + SurfaceOffset(mapArrowHeightOffset);
            activeMapArrows.Add(Instantiate(mapArrowPrefab, spawnPos, rotation, parent));
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
        Quaternion tiltMapPlaneToWorldUp = Quaternion.FromToRotation(grid.MapNormal, Vector3.up);
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
                * Quaternion.LookRotation(direction, Vector3.up)
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
        if (distanceTMP == null)
            return;

        distanceTMP.enabled = true;
        distanceTMP.text = $"Distance: {GetPathDistanceFeet():F1} ft";

        Canvas canvas = distanceTMP.canvas;
        if (canvas != null)
        {
            canvas.enabled = true;
            if (!canvas.gameObject.activeInHierarchy)
                canvas.gameObject.SetActive(true);
            if (canvas.renderMode == RenderMode.WorldSpace || canvas.renderMode == RenderMode.ScreenSpaceCamera)
            {
                Camera cam = ResolveDistanceCanvasCamera();
                if (cam != null && canvas.worldCamera != cam)
                    canvas.worldCamera = cam;
                else if (cam == null && !distanceUiCameraWarningLogged)
                {
                    distanceUiCameraWarningLogged = true;
                    Debug.LogWarning(
                        "PathTest: No camera found for distance UI canvas. Assign PathTest.distanceUICamera to your HoloLens/XR rig camera so the distance text renders in player builds.");
                }
            }
        }

        if (!distanceTMP.gameObject.activeInHierarchy)
            distanceTMP.gameObject.SetActive(true);

        distanceTMP.ForceMeshUpdate(true);
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
