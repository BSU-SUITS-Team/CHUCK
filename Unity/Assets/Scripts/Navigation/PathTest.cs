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
    [Tooltip("Optional tracked head/root transform for movement + world-arrow anchoring. If unset, falls back to player, then a live camera transform.")]
    public Transform trackingTransformOverride;
    public Transform mapStartPoint;
    public Transform endPoint;
    [Tooltip("Optional world-space icon at the route goal, placed above the floor path (same layout as world arrows). Assign e.g. EndPointMarker.")]
    public Transform endPointWorldMarker;
    [Tooltip("If set, the floor path is the map A* shape scaled & rotated to run from your feet (tracking) to this transform’s XZ. Leave empty to use map north / yaw only (no fixed world goal).")]
    public Transform worldRouteGoalWorld;
    [Tooltip("Extra meters along world up above the computed world goal position.")]
    public float endPointMarkerHeightAboveRoute = 0.35f;
    [Tooltip("Multiplier applied only to endPointWorldMarker scale in world space marker mode. Does not affect map markers.")]
    public float endPointWorldMarkerScaleMultiplier = 1f;
    [Tooltip("If the marker's authored localScale is near zero, this fallback base scale is used before applying the multiplier.")]
    public Vector3 endPointWorldMarkerFallbackBaseScale = Vector3.one;
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
    [Tooltip("If off (default), the distance label follows the green→red journey (same node order as the map line / arrows when calibration is active). If on, sums edges in raw currentPath list order — with Invert Path Visualization, that can add a long first chord (player → wrong end) and inflate the number.")]
    public bool distanceSumUsesRawPathListOrder = false;
    public float repathIntervalSeconds = 0.4f;
    [Tooltip("Minimum player translation (meters) before a periodic repath runs. Reduces work when standing still.")]
    public float repathMinPlayerMoveMeters = 0.08f;
    public bool regenerateGridEachRepath = true;
    public int nearestWalkableSearchRadius = 20;
    [Header("Distance display")]
    [Tooltip("When remaining route distance is at or below this value (feet), the UI shows destinationReachedText instead of a numeric distance.")]
    public float destinationReachedFeet = 10f;
    [Tooltip("Shown when remaining distance ≤ destinationReachedFeet (path session active).")]
    public string destinationReachedText = "Destination reached";

    [Header("Visualization — line")]
    public LineRenderer pathLine;
    [Tooltip("When off (default), the LineRenderer is cleared and disabled so only map/world arrows show the route.")]
    public bool showMapPathLine = false;
    public float lineHeightOffset = 0.01f;
    [Tooltip("Only used when Show Map Path Line is on and A* returns no path: draw a straight segment between snapped start/end.")]
    public bool drawDirectLineWhenNoPath = false;

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
    [Tooltip("Adds 180° to the world floor route yaw before North lock takes over. Use as a one-click flip when the default polyline points the opposite way from your goal. After locking North to the correct heading, turn this off.")]
    public bool flipWorldFloorDefaultDirection = false;
    [Tooltip("After A*, reverse the node list. Use only if the route still runs goal→start (same geometry, wrong direction along it). Map arrows follow this order; world floor arrows always use start→goal so the trail matches green→red on the chart.")]
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
    [Tooltip("Build the world floor route directly from grid offsets (default, recommended). Guarantees the shape matches the map A* path even if the map panel moved/rotated since calibration. Turn off to use the legacy tilt-then-project pipeline (can distort shape on tilted or moving map panels).")]
    public bool worldFloorUsesGridLayout = true;
    [Tooltip("When a path exists, snap the tracking transform’s Y rotation toward the first floor segment. Off by default so the world route stays independent of head pose.")]
    public bool snapPlayerYawToWorldPath = false;

    [Header("HUD — off-screen destination turn cue")]
    [Tooltip("Defaults to worldArrowPrefab when empty. NavArrow mesh parented to the headset camera when the goal is outside the view frustum.")]
    public GameObject offScreenTurnCuePrefab;
    [Tooltip("Shrink the “visible” viewport by this fraction on each side (0 = use full screen).")]
    [Range(0f, 0.45f)]
    public float offScreenTurnCueViewportMargin = 0.04f;
    [Tooltip("Meters in front of the camera (local +Z) for the cue.")]
    public float offScreenTurnCueLocalDepth = 1.2f;
    [Tooltip("Meters left/right of view center (camera local ±X).")]
    public float offScreenTurnCueLocalX = 0.38f;
    [Tooltip("Meters up/down from view center (camera local Y).")]
    public float offScreenTurnCueLocalY = -0.06f;
    [Tooltip("Local scale applied to the cue instance.")]
    public Vector3 offScreenTurnCueLocalScale = new Vector3(0.6f, 0.6f, 0.6f);
    [Tooltip("Extra local rotation after aiming the mesh along screen left/right (degrees).")]
    public Vector3 offScreenTurnCueRotationExtraEuler = Vector3.zero;

    [Header("Markers & map lift")]
    [Tooltip("Meters along the map RectTransform forward (out of the image). Same for green, red, line, arrows. 0 = on the quad.")]
    public float markerHeightOffset = 0f;
    [Tooltip("Negates the forward lift.")]
    public bool invertSurfaceOffsetDirection = false;

    [Header("North / world compass (map is visual only for users; grid pathfinding is unchanged)")]
    [Tooltip("Optional override for which way is “north” on the map plane. Leave empty when the map is laid out with chart north = image up / centered: north is then +grid Y from the map center (see code). Only use this if you need a custom direction (e.g. odd UV mirroring).")]
    public Transform mapNorthReference;
    [Tooltip("If mapNorthReference is set, disables Renderers under it so it can stay invisible in headset.")]
    public bool hideMapNorthReferenceRenderers = true;
    [Tooltip("After Orient North, world route yaw is frozen to that heading vs map north. When off, world route uses chart→floor tilt only (no continuous head coupling).")]
    [SerializeField]
    private bool northOrientationLocked;
    [SerializeField]
    private float lockedNorthYawDegrees;

    /// <summary>Map tilt + north locked on session start / Orient North. Floor path is rebuilt each repath from the map polyline.</summary>
    private bool _worldCalibCaptured;
    private Vector3 _worldCalibAnchor;
    private bool _worldCalibIsRect;
    private Vector3 _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3;
    private Vector3 _worldCalibCenter;
    private Vector3 _worldCalibAxisX, _worldCalibAxisY;
    private float _worldCalibMapW, _worldCalibMapH;
    private int _worldCalibGw, _worldCalibGh;
    private Quaternion _worldCalibTiltToHorizontal;
    private float _worldCalibNorthYawDeg;

    private bool _worldLayoutValid;
    private int _snapStartX, _snapStartY;
    private readonly List<Vector3> _snapPathMapPoints = new List<Vector3>();
    private Vector3 _snapMapStartWorld;
    private readonly List<Vector3> _worldPathFloorPositions = new List<Vector3>();
    private readonly List<Node> _snapPathNodes = new List<Node>();

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
    private Vector3 repathAnchorWorld;
    private Vector3 navigationRootInitialLocalPos;
    private bool navigationRootBaselineStored;
    private bool distanceUiCameraWarningLogged;
    private bool distanceOverlayFixLogged;
    private bool endPointMarkerScaleWarningLogged;
    private Transform cachedEndPointWorldMarker;
    private Vector3 cachedEndPointWorldMarkerBaseLocalScale;
    private bool mapNorthReferenceRenderersHidden;

    private GameObject _offScreenCueLeft;
    private GameObject _offScreenCueRight;

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
        DestroyOffScreenTurnCueInstances();
        if (Instance == this)
            Instance = null;
    }

    public bool IsPathSessionActive() => pathSessionActive;

    public bool IsNorthOrientationLocked => northOrientationLocked;

    /// <summary>
    /// Stored at Orient North: world-up yaw (degrees) applied when laying the map route on the floor so chart north
    /// matched your horizontal look direction at press. Drives world-arrow and endpoint layout via spatial calibration.
    /// </summary>
    public float NorthCalibrationYawDegrees => northOrientationLocked ? lockedNorthYawDegrees : 0f;

    /// <summary>
    /// Horizontal world direction of chart “north” on the map quad (from <see cref="mapNorthReference"/> or grid +Y). Does not include Orient North offset.
    /// </summary>
    public Vector3 GetChartNorthWorldDirectionHorizontal() => GetMapNorthDirectionWorldHorizontal(true);

    /// <summary>
    /// Treat your current horizontal gaze as “facing map north” in world space: stores a yaw offset so floor arrows and
    /// the world endpoint marker map chart geometry using that north→world alignment. Does not start or stop pathfinding;
    /// if a path session is already active, performs a normal repath so all visuals and distance reflect the new mapping.
    /// </summary>
    public void CalibrateNorthFromUserFacing()
    {
        Vector3 mapN = GetMapNorthDirectionWorldHorizontal(false);
        Transform tr = ResolveTrackingTransform();
        if (tr == null)
            return;
        Vector3 userF = Vector3.ProjectOnPlane(tr.forward, Vector3.up);
        if (userF.sqrMagnitude < 1e-8f)
            return;
        userF.Normalize();
        // World yaw to apply so projected map “north” matches the direction you were looking when you pressed Orient North.
        lockedNorthYawDegrees = Vector3.SignedAngle(mapN, userF, Vector3.up);
        northOrientationLocked = true;
        CaptureWorldSpatialCalibration();
        if (pathSessionActive)
            ForceRepath();
    }

    /// <summary>Clear Orient North lock; world route yaw returns to chart-only (no locked heading).</summary>
    public void ClearNorthOrientationLock()
    {
        northOrientationLocked = false;
        if (pathSessionActive)
        {
            CaptureWorldSpatialCalibration();
            ForceRepath();
        }
    }

    /// <summary>Re-run north-marker hiding after <see cref="mapNorthReference"/> is assigned at runtime (e.g. from Navigation prefab).</summary>
    public void RefreshMapNorthReferenceVisibility()
    {
        mapNorthReferenceRenderersHidden = false;
        ApplyMapNorthReferenceVisibility();
    }

    private Transform ResolveTrackingTransform()
    {
        if (trackingTransformOverride != null)
            return trackingTransformOverride;
        if (distanceUICamera != null && distanceUICamera.isActiveAndEnabled)
            return distanceUICamera.transform;
        if (Camera.main != null && Camera.main.isActiveAndEnabled)
            return Camera.main.transform;
        if (player != null)
            return player;
        Camera cam = ResolveDistanceCanvasCamera();
        return cam != null ? cam.transform : null;
    }

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
        Transform tracking = ResolveTrackingTransform();
        if (tracking != null)
            repathAnchorWorld = tracking.position;
        if (!active)
            ClearPathVisualization();
        else
        {
            TryRefreshNavigationMapBinding();
            if (tracking != null)
                lastPlayerWorldPos = tracking.position;
            if (mapStartPoint != null)
                mapStartPoint.gameObject.SetActive(true);
            if (endPoint != null)
                endPoint.gameObject.SetActive(true);
            CaptureWorldSpatialCalibration();
            ForceRepath();
        }
    }

    /// <summary>Rebinds GridManager to the Navigation panel that is currently open so map markers match the menu (not the scene prefab pose).</summary>
    public void TryRefreshNavigationMapBinding()
    {
        Navigation.RefreshPathfindingMapBindingStatic();
    }

    private void ClearPathVisualization()
    {
        currentPath.Clear();
        _worldLayoutValid = false;
        _worldCalibCaptured = false;
        _snapPathMapPoints.Clear();
        _snapPathNodes.Clear();
        _worldPathFloorPositions.Clear();
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
        if (endPointWorldMarker != null)
            endPointWorldMarker.gameObject.SetActive(false);
        HideOffScreenTurnCues();
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
        if (grid == null || pathfinder == null)
        {
            Debug.LogWarning("PathTest is missing required references.");
            enabled = false;
            return;
        }

        currentCoordinate = startCoordinate;
        Transform tracking = ResolveTrackingTransform();
        if (tracking != null)
        {
            lastPlayerWorldPos = tracking.position;
            repathAnchorWorld = tracking.position;
        }
        else
        {
            lastPlayerWorldPos = Vector3.zero;
            repathAnchorWorld = Vector3.zero;
            Debug.LogWarning("PathTest: No tracking transform found. Assign player or trackingTransformOverride for live start/distance/repath updates on device.");
        }

        if (grid.grid == null)
            grid.GenerateGrid();

        if (pathLine != null)
        {
            pathLine.useWorldSpace = true;
            if (!showMapPathLine)
            {
                pathLine.positionCount = 0;
                pathLine.enabled = false;
            }
        }

        if (!pathSessionActive)
            ClearPathVisualization();

        ApplyMapNorthReferenceVisibility();
    }

    private void ApplyMapNorthReferenceVisibility()
    {
        if (mapNorthReference == null || !hideMapNorthReferenceRenderers || mapNorthReferenceRenderersHidden)
            return;
        var renderers = mapNorthReference.GetComponentsInChildren<Renderer>(true);
        for (int i = 0; i < renderers.Length; i++)
            renderers[i].enabled = false;
        mapNorthReferenceRenderersHidden = true;
    }

    /// <summary>
    /// Map tangents flattened with the same tilt-to-horizontal rotation as the floor route (tilt quad normal → world up).
    /// Using <see cref="Vector3.ProjectOnPlane"/> onto world Y instead disagrees with that on tilted maps and reads as XZ mirror vs the floor path.
    /// </summary>
    private Vector3 MapPlaneTangentToHorizontalWorld(Vector3 tangentWorld, bool preferCapturedTilt)
    {
        if (grid == null || tangentWorld.sqrMagnitude < 1e-12f)
            return Vector3.forward;

        Quaternion tilt = preferCapturedTilt && _worldCalibCaptured
            ? _worldCalibTiltToHorizontal
            : Quaternion.FromToRotation(grid.GetGridPlaneNormal(), Vector3.up);
        Vector3 h = Vector3.ProjectOnPlane(tilt * tangentWorld, Vector3.up);
        if (h.sqrMagnitude < 1e-8f)
            return Vector3.forward;
        return h.normalized;
    }

    /// <summary>
    /// Horizontal world direction for “map north”: default is +grid Y from map center (map image “up” when chart Y matches image up).
    /// Optional <see cref="mapNorthReference"/> overrides by using the direction center → ref on the map plane. Does not affect A*; only world arrow/marker yaw vs head.
    /// </summary>
    /// <param name="preferCapturedTilt">When true and calibration exists, use the same tilt as world arrows (stable while walking). When false, use the live grid plane (e.g. Orient North before re-capture).</param>
    private Vector3 GetMapNorthDirectionWorldHorizontal(bool preferCapturedTilt)
    {
        if (grid == null)
            return Vector3.forward;

        Vector3 center = grid.SnapOntoVisualMapFace(grid.MapPlaneCenter);
        if (mapNorthReference != null)
        {
            Vector3 refOnPlane = grid.SnapOntoVisualMapFace(mapNorthReference.position);
            Vector3 d = refOnPlane - center;
            return MapPlaneTangentToHorizontalWorld(d, preferCapturedTilt);
        }

        int mx = Mathf.Clamp(grid.GridWidth / 2, 0, grid.GridWidth - 1);
        int my = Mathf.Clamp(grid.GridHeight / 2, 0, grid.GridHeight - 1);
        int ny = Mathf.Min(grid.GridHeight - 1, my + 1);
        Vector3 c = grid.GridToWorld(mx, my);
        Vector3 n = grid.GridToWorld(mx, ny);
        return MapPlaneTangentToHorizontalWorld(n - c, preferCapturedTilt);
    }

    /// <summary>World-route yaw (degrees, world up): locked offset after Orient North, otherwise 0 so the trail does not spin with your head while the map moves.</summary>
    private float ComputeNorthAlignmentYawDegrees()
    {
        return northOrientationLocked ? lockedNorthYawDegrees : 0f;
    }

    /// <summary>
    /// World-space chart north used for user movement projection. When Orient North is locked, this is map-north rotated by the stored yaw.
    /// </summary>
    private Vector3 GetCalibratedChartNorthWorldHorizontal()
    {
        Vector3 mapNorth = GetMapNorthDirectionWorldHorizontal(true);
        if (northOrientationLocked)
            mapNorth = Quaternion.Euler(0f, lockedNorthYawDegrees, 0f) * mapNorth;
        mapNorth = Vector3.ProjectOnPlane(mapNorth, Vector3.up);
        if (mapNorth.sqrMagnitude < 1e-8f)
            return Vector3.forward;
        return mapNorth.normalized;
    }

    private void Update()
    {
        UpdateCoordinateFromMovement();

        if (!pathSessionActive)
            return;

        repathTimer += Time.deltaTime;
        bool intervalElapsed = repathTimer >= Mathf.Max(0.05f, repathIntervalSeconds);
        if (intervalElapsed)
        {
            repathTimer = 0f;
            Transform tracking = ResolveTrackingTransform();
            if (tracking != null)
                repathAnchorWorld = tracking.position;
            ForceRepath();
        }

        UpdateDistanceUI();
    }

    private void LateUpdate()
    {
        if (!pathSessionActive || grid == null || grid.grid == null)
        {
            HideOffScreenTurnCues();
            return;
        }

        ResolveStartEndGrid(out Vector2Int rs, out Vector2Int re);

        if (!IsMapSurfaceVisible())
        {
            SetMapVisualsVisible(false);
            UpdateEndPointWorldMarker();
            UpdateOffScreenTurnCues();
            return;
        }

        if (mapStartPoint == null && endPoint == null && endPointWorldMarker == null)
        {
            UpdateEndPointWorldMarker();
            UpdateOffScreenTurnCues();
            return;
        }

        SetMapVisualsVisible(true);
        PlaceEndpointMarkers(rs, re);
        UpdateEndPointWorldMarker();

        // Map arrows are parented to the moving map; pathLine is world-space and must be refreshed when it is used.
        if (pathLine != null && pathSessionActive && showMapPathLine)
            DrawPathLine();

        UpdateOffScreenTurnCues();
    }

    private bool IsMapSurfaceVisible()
    {
        if (grid == null || grid.ActiveMapTransform == null)
            return false;
        return grid.ActiveMapTransform.gameObject.activeInHierarchy;
    }

    private void SetMapVisualsVisible(bool visible)
    {
        if (mapStartPoint != null)
            mapStartPoint.gameObject.SetActive(visible && pathSessionActive);
        if (endPoint != null)
            endPoint.gameObject.SetActive(visible && pathSessionActive);
        if (!visible)
        {
            DestroyArrowList(activeMapArrows);
            if (pathLine != null)
            {
                pathLine.positionCount = 0;
                pathLine.enabled = false;
            }
        }
        else if (pathLine != null)
        {
            if (showMapPathLine)
                pathLine.enabled = true;
            else
            {
                pathLine.positionCount = 0;
                pathLine.enabled = false;
            }
        }
    }

    private void UpdateCoordinateFromMovement()
    {
        Transform tracking = ResolveTrackingTransform();
        if (tracking == null)
            return;
        Vector3 delta = tracking.position - lastPlayerWorldPos;
        lastPlayerWorldPos = tracking.position;
        Vector3 deltaFlat = Vector3.ProjectOnPlane(delta, Vector3.up);
        if (deltaFlat.sqrMagnitude <= Mathf.Epsilon)
            return;

        Vector3 north = GetCalibratedChartNorthWorldHorizontal();
        Vector3 east = Vector3.Cross(Vector3.up, north);
        if (east.sqrMagnitude < 1e-8f)
            east = Vector3.right;
        else
            east.Normalize();

        float eastFeet = Vector3.Dot(deltaFlat, east) * feetPerMeter;
        float northFeet = Vector3.Dot(deltaFlat, north) * feetPerMeter;
        currentCoordinate.x += eastFeet;
        currentCoordinate.y += northFeet;
    }

    /// <summary>
    /// Map start/end in grid cells. Start uses <see cref="currentCoordinate"/> (chart). World floor visuals anchor at each repath (see snapshot), not every frame from the live map pose.
    /// </summary>
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

    private void UpdateEndPointWorldMarker()
    {
        if (endPointWorldMarker == null)
            return;
        if (!pathSessionActive || !_worldCalibCaptured || !_worldLayoutValid)
        {
            endPointWorldMarker.gameObject.SetActive(false);
            return;
        }

        endPointWorldMarker.gameObject.SetActive(true);

        if (worldRouteGoalWorld != null && endPointWorldMarker == worldRouteGoalWorld)
        {
            ApplyEndPointWorldMarkerScale();
            return;
        }

        Vector3 pos;
        if (_worldPathFloorPositions.Count > 0)
            pos = _worldPathFloorPositions[_worldPathFloorPositions.Count - 1] + Vector3.up * worldArrowHeightAbovePlayer;
        else if (_snapPathMapPoints.Count > 0)
            pos = WorldArrowFloorFromSnapshotMapPoint(_snapPathMapPoints[_snapPathMapPoints.Count - 1]);
        else
            pos = WorldArrowFloorFromSnapshot(_lineResolvedEnd.x, _lineResolvedEnd.y);
        pos += Vector3.up * endPointMarkerHeightAboveRoute;
        endPointWorldMarker.position = pos;
        ApplyEndPointWorldMarkerScale();
    }

    private void ApplyEndPointWorldMarkerScale()
    {
        if (endPointWorldMarker == null)
            return;

        if (cachedEndPointWorldMarker != endPointWorldMarker)
        {
            cachedEndPointWorldMarker = endPointWorldMarker;
            cachedEndPointWorldMarkerBaseLocalScale = endPointWorldMarker.localScale;
        }

        Vector3 baseScale = cachedEndPointWorldMarkerBaseLocalScale;
        if (baseScale.sqrMagnitude <= 1e-10f)
        {
            baseScale = endPointWorldMarkerFallbackBaseScale.sqrMagnitude > 1e-10f
                ? endPointWorldMarkerFallbackBaseScale
                : Vector3.one;
            if (!endPointMarkerScaleWarningLogged)
            {
                endPointMarkerScaleWarningLogged = true;
                Debug.LogWarning("PathTest: EndPointWorldMarker base scale is near zero. Using fallback base scale for world marker visibility.");
            }
        }

        float s = Mathf.Max(0.01f, endPointWorldMarkerScaleMultiplier);
        endPointWorldMarker.localScale = baseScale * s;
    }

    private void ForceRepath()
    {
        if (regenerateGridEachRepath || grid.grid == null)
            grid.GenerateGrid();

        if (grid.grid == null)
            return;

        ResolveStartEndGrid(out Vector2Int resolvedStart, out Vector2Int resolvedEnd);
        bool mapVisible = IsMapSurfaceVisible();
        SetMapVisualsVisible(mapVisible);
        if (mapVisible)
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

        // One journey-ordered snap for map line, map arrows, and world floor (fixes mismatch when invertPathVisualizationOrder reverses currentPath).
        RefreshWorldRouteLayoutSnapshot();

        if (mapVisible)
        {
            DrawPathLine();
            RebuildMapArrows();
        }
        RebuildWorldArrows();
    }

    /// <summary>
    /// Journey index 0 = route start (green), Count-1 = goal (red). When <see cref="invertPathVisualizationOrder"/> is on,
    /// <see cref="currentPath"/> is stored tail-first; this maps a journey index to the correct list slot.
    /// </summary>
    private Node GetCurrentPathNodeJourneyOrder(int journeyIndex)
    {
        if (invertPathVisualizationOrder)
            return currentPath[currentPath.Count - 1 - journeyIndex];
        return currentPath[journeyIndex];
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
        if (!showMapPathLine)
        {
            pathLine.positionCount = 0;
            pathLine.enabled = false;
            return;
        }

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

        if (_snapPathMapPoints.Count == currentPath.Count && _snapPathMapPoints.Count > 0)
        {
            pathLine.positionCount = _snapPathMapPoints.Count + 1;
            pathLine.SetPosition(0, start);
            for (int i = 0; i < _snapPathMapPoints.Count; i++)
                pathLine.SetPosition(i + 1, _snapPathMapPoints[i] + lineLift);
            return;
        }

        pathLine.positionCount = currentPath.Count + 1;
        pathLine.SetPosition(0, start);
        for (int i = 0; i < currentPath.Count; i++)
        {
            Node n = GetCurrentPathNodeJourneyOrder(i);
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

        bool useJourneySnap = _snapPathNodes.Count == currentPath.Count
            && _snapPathMapPoints.Count == _snapPathNodes.Count
            && _snapPathNodes.Count >= 2;

        Transform parent = EnsureMapArrowsRuntimeRoot();
        int step = Mathf.Max(1, mapArrowEveryNNodes);
        float arrowAmount = mapArrowHeightOffset >= 0f ? mapArrowHeightOffset : lineHeightOffset;
        Vector3 arrowLift = MapFaceLift(arrowAmount);
        Vector3 faceNormal = grid.MapFaceOut;

        int pathCount = useJourneySnap ? _snapPathNodes.Count : currentPath.Count;
        for (int i = 0; i < pathCount - 1; i += step)
        {
            Node current = useJourneySnap ? _snapPathNodes[i] : GetCurrentPathNodeJourneyOrder(i);
            int nextIdx = Mathf.Min(i + 1, pathCount - 1);
            Node next = useJourneySnap ? _snapPathNodes[nextIdx] : GetCurrentPathNodeJourneyOrder(nextIdx);

            Vector3 currentPos = useJourneySnap
                ? _snapPathMapPoints[i]
                : grid.SnapOntoVisualMapFace(grid.GridToWorld(current.x, current.y));
            Vector3 nextPos = useJourneySnap
                ? _snapPathMapPoints[nextIdx]
                : grid.SnapOntoVisualMapFace(grid.GridToWorld(next.x, next.y));
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

    /// <summary>Call when starting a path session or pressing Orient North. Locks world anchor + map basis + north; timer repath does not move the trail with the user.</summary>
    private void CaptureWorldSpatialCalibration()
    {
        _worldCalibCaptured = false;
        _worldLayoutValid = false;
        if (grid == null || grid.ActiveMapTransform == null)
            return;

        Transform tr = ResolveTrackingTransform();
        _worldCalibAnchor = tr != null ? tr.position : Vector3.zero;
        if (player != null)
            _worldCalibAnchor.y = player.position.y;
        else if (worldArrowParent != null)
            _worldCalibAnchor.y = worldArrowParent.position.y;

        var corners = new Vector3[4];
        if (grid.TryCopyRectWorldCorners(corners))
        {
            _worldCalibIsRect = true;
            _worldCalibCorner0 = corners[0];
            _worldCalibCorner1 = corners[1];
            _worldCalibCorner3 = corners[3];
        }
        else
        {
            _worldCalibIsRect = false;
            _worldCalibCenter = grid.ActiveMapTransform.position + grid.MapAxisX * grid.mapCenterOffset.x + grid.MapAxisY * grid.mapCenterOffset.y;
            _worldCalibAxisX = grid.MapAxisX;
            _worldCalibAxisY = grid.MapAxisY;
            _worldCalibMapW = grid.mapWidth;
            _worldCalibMapH = grid.mapHeight;
            _worldCalibGw = grid.GridWidth;
            _worldCalibGh = grid.GridHeight;
        }

        _worldCalibTiltToHorizontal = Quaternion.FromToRotation(grid.GetGridPlaneNormal(), Vector3.up);
        _worldCalibNorthYawDeg = ComputeNorthAlignmentYawDegrees();
        _worldCalibCaptured = true;
    }

    private Vector3 GetWorldPathFloorAnchor()
    {
        Transform tr = ResolveTrackingTransform();
        Vector3 a = tr != null ? tr.position : _worldCalibAnchor;
        if (player != null)
            a.y = player.position.y;
        else if (worldArrowParent != null)
            a.y = worldArrowParent.position.y;
        return a;
    }

    /// <summary>Per repath: snapshot the current map polyline so world arrows follow the same route geometry.</summary>
    private void RefreshWorldRouteLayoutSnapshot()
    {
        _worldLayoutValid = false;
        _snapPathMapPoints.Clear();
        _snapPathNodes.Clear();
        if (grid == null || currentPath.Count < 1 || !_worldCalibCaptured)
            return;

        for (int i = 0; i < currentPath.Count; i++)
        {
            Node n = currentPath[i];
            _snapPathNodes.Add(n);
            Vector3 mapPoint = grid.SnapOntoVisualMapFace(grid.GridToWorld(n.x, n.y));
            _snapPathMapPoints.Add(mapPoint);
        }

        // Journey must be green (resolved start) → red (resolved end). Prefer matching path endpoints by grid cell —
        // world-space distance ties could flip the polyline the wrong way and break map line vs floor route.
        if (_snapPathMapPoints.Count >= 2)
        {
            Node head = _snapPathNodes[0];
            Node tail = _snapPathNodes[_snapPathMapPoints.Count - 1];
            bool headIsStart = head.x == _lineResolvedStart.x && head.y == _lineResolvedStart.y;
            bool tailIsEnd = tail.x == _lineResolvedEnd.x && tail.y == _lineResolvedEnd.y;
            if (!(headIsStart && tailIsEnd))
            {
                bool headIsEnd = head.x == _lineResolvedEnd.x && head.y == _lineResolvedEnd.y;
                bool tailIsStart = tail.x == _lineResolvedStart.x && tail.y == _lineResolvedStart.y;
                if (headIsEnd && tailIsStart)
                {
                    _snapPathMapPoints.Reverse();
                    _snapPathNodes.Reverse();
                }
                else
                {
                    Vector3 startSnap = grid.SnapOntoVisualMapFace(grid.GridToWorld(_lineResolvedStart.x, _lineResolvedStart.y));
                    Vector3 endSnap = grid.SnapOntoVisualMapFace(grid.GridToWorld(_lineResolvedEnd.x, _lineResolvedEnd.y));
                    int last = _snapPathMapPoints.Count - 1;
                    float forwardCost =
                        (_snapPathMapPoints[0] - startSnap).sqrMagnitude +
                        (_snapPathMapPoints[last] - endSnap).sqrMagnitude;
                    float reversedCost =
                        (_snapPathMapPoints[last] - startSnap).sqrMagnitude +
                        (_snapPathMapPoints[0] - endSnap).sqrMagnitude;
                    if (reversedCost < forwardCost)
                    {
                        _snapPathMapPoints.Reverse();
                        _snapPathNodes.Reverse();
                    }
                }
            }
        }

        _snapStartX = _lineResolvedStart.x;
        _snapStartY = _lineResolvedStart.y;
        _snapMapStartWorld = _snapPathMapPoints[0];

        _worldPathFloorPositions.Clear();
        Vector3 w0 = GetWorldPathFloorAnchor();
        float spaceScale = Mathf.Max(0.001f, worldPathUniformSpaceScale);

        var rawFlat = new List<Vector3>(_snapPathMapPoints.Count);
        if (worldFloorUsesGridLayout)
        {
            // Use grid offsets so the floor polyline shape == map A* shape 1:1, independent of the map panel's
            // current 3D orientation (legacy tilt+project distorted shape when the map moved after calibration).
            Vector3 northDir = GetCalibratedChartNorthWorldHorizontal();
            if (northDir.sqrMagnitude < 1e-8f) northDir = Vector3.forward; else northDir.Normalize();
            Vector3 eastDir = Vector3.Cross(Vector3.up, northDir);
            if (eastDir.sqrMagnitude < 1e-8f) eastDir = Vector3.right; else eastDir.Normalize();

            Node startNode = _snapPathNodes[0];
            float cellW = Mathf.Max(1e-6f, grid.CellWidth);
            float cellH = Mathf.Max(1e-6f, grid.CellHeight);
            for (int i = 0; i < _snapPathNodes.Count; i++)
            {
                Node n = _snapPathNodes[i];
                float gx = (n.x - startNode.x) * cellW * spaceScale;
                float gy = (n.y - startNode.y) * cellH * spaceScale;
                rawFlat.Add(eastDir * gx + northDir * gy);
            }
        }
        else
        {
            for (int i = 0; i < _snapPathMapPoints.Count; i++)
            {
                Vector3 d = _snapPathMapPoints[i] - _snapPathMapPoints[0];
                Vector3 flat = Vector3.ProjectOnPlane(_worldCalibTiltToHorizontal * d * spaceScale, Vector3.up);
                rawFlat.Add(flat);
            }
        }

        Vector3 w1;
        if (worldRouteGoalWorld != null)
        {
            w1 = worldRouteGoalWorld.position;
            w1.y = w0.y;
        }
        else
        {
            // Grid layout already lives in calibrated north basis — only apply the user's extra yaw offset.
            // Legacy tilt mode lacks that pre-alignment, so it still needs the captured north yaw.
            float extraYaw = worldFloorUsesGridLayout
                ? worldPathYawOffsetDegrees
                : worldPathYawOffsetDegrees + _worldCalibNorthYawDeg;
            if (flipWorldFloorDefaultDirection)
                extraYaw += 180f;
            Quaternion qNorth = Quaternion.Euler(0f, extraYaw, 0f);
            w1 = w0 + qNorth * rawFlat[rawFlat.Count - 1];
        }

        Vector3 target = Vector3.ProjectOnPlane(w1 - w0, Vector3.up);
        Vector3 source = rawFlat[rawFlat.Count - 1];
        if (source.sqrMagnitude < 1e-14f)
        {
            for (int i = 0; i < rawFlat.Count; i++)
                _worldPathFloorPositions.Add(w0);
        }
        else
        {
            float stretch = target.magnitude / source.magnitude;
            float spin = Vector3.SignedAngle(source, target, Vector3.up);
            Quaternion spinQ = Quaternion.AngleAxis(spin, Vector3.up);
            for (int i = 0; i < rawFlat.Count; i++)
                _worldPathFloorPositions.Add(w0 + spinQ * (rawFlat[i] * stretch));
        }

        _worldLayoutValid = true;
    }

    private Vector3 GridToWorldSnapshot(int gx, int gy)
    {
        if (!_worldCalibCaptured)
            return _worldCalibAnchor;
        if (_worldCalibIsRect)
            return grid.GridToWorldUsingRectCorners(gx, gy, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
        float cw = _worldCalibMapW / Mathf.Max(1, _worldCalibGw);
        float ch = _worldCalibMapH / Mathf.Max(1, _worldCalibGh);
        Vector3 bl = _worldCalibCenter - _worldCalibAxisX * (_worldCalibMapW * 0.5f) - _worldCalibAxisY * (_worldCalibMapH * 0.5f);
        return bl + _worldCalibAxisX * ((gx + 0.5f) * cw) + _worldCalibAxisY * ((gy + 0.5f) * ch);
    }

    /// <summary>Nearest point on the current floor route for a grid cell (map face).</summary>
    private Vector3 WorldArrowFloorFromSnapshot(int gx, int gy)
    {
        if (grid == null || !_worldLayoutValid || !_worldCalibCaptured)
            return GetWorldPathFloorAnchor() + Vector3.up * worldArrowHeightAbovePlayer;
        Vector3 p = grid.SnapOntoVisualMapFace(grid.GridToWorld(gx, gy));
        return WorldArrowFloorFromSnapshotMapPoint(p);
    }

    /// <summary>Nearest vertex on the snapped route polyline → precomputed floor position.</summary>
    private Vector3 WorldArrowFloorFromSnapshotMapPoint(Vector3 mapPointWorld)
    {
        if (!_worldLayoutValid || !_worldCalibCaptured || _worldPathFloorPositions.Count != _snapPathMapPoints.Count)
            return GetWorldPathFloorAnchor() + Vector3.up * worldArrowHeightAbovePlayer;
        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < _snapPathMapPoints.Count; i++)
        {
            float d = (_snapPathMapPoints[i] - mapPointWorld).sqrMagnitude;
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }

        return _worldPathFloorPositions[best] + Vector3.up * worldArrowHeightAbovePlayer;
    }

    private void RebuildWorldArrows()
    {
        DestroyArrowList(activeWorldArrows);
        if (!_worldLayoutValid || _snapPathMapPoints.Count != currentPath.Count || currentPath.Count < 1)
            RefreshWorldRouteLayoutSnapshot();

        if (worldArrowPrefab == null || grid == null || grid.grid == null || currentPath.Count < 2 || !_worldCalibCaptured || !_worldLayoutValid
            || _snapPathMapPoints.Count < 2 || _worldPathFloorPositions.Count != _snapPathMapPoints.Count)
            return;

        float spaceScale = Mathf.Max(0.001f, worldPathUniformSpaceScale);
        Vector3 visualScaleMul = worldArrowMeshScalesWithPath
            ? Vector3.Scale(worldArrowScaleMultiplier, new Vector3(spaceScale, spaceScale, spaceScale))
            : worldArrowScaleMultiplier;

        // Same placement cadence as map arrows (every pathStep). Heading aims at the next placed arrow (i + pathStep);
        // using i→i+1 made sparse arrows point wrong vs the visible trail.
        int pathStep = Mathf.Max(1, mapArrowEveryNNodes);
        Quaternion playerMeshYawOnly = Quaternion.Euler(0f, worldArrowLookYawOffsetDegrees, 0f);
        Transform tracking = ResolveTrackingTransform();

        if (snapPlayerYawToWorldPath && _worldPathFloorPositions.Count >= 2 && tracking != null)
        {
            int aim = Mathf.Min(pathStep, _worldPathFloorPositions.Count - 1);
            Vector3 p0 = _worldPathFloorPositions[0] + Vector3.up * worldArrowHeightAbovePlayer;
            Vector3 p1 = _worldPathFloorPositions[aim] + Vector3.up * worldArrowHeightAbovePlayer;
            Vector3 flat = Vector3.ProjectOnPlane(p1 - p0, Vector3.up);
            if (flat.sqrMagnitude > 1e-8f)
            {
                Quaternion facePath = Quaternion.LookRotation(flat.normalized, Vector3.up) * playerMeshYawOnly;
                Vector3 e = tracking.eulerAngles;
                e.y = facePath.eulerAngles.y;
                tracking.eulerAngles = e;
            }
        }

        for (int i = 0; i < _snapPathMapPoints.Count - 1; i += pathStep)
        {
            // Must aim at the next *placed* arrow (i + pathStep). Using i+1 made headings follow single grid
            // steps while arrows are spaced wider — they visibly ignored the next breadcrumb on curves.
            int nextIdx = Mathf.Min(i + pathStep, _snapPathMapPoints.Count - 1);
            Vector3 lift = Vector3.up * worldArrowHeightAbovePlayer;
            Vector3 worldPos = _worldPathFloorPositions[i] + lift;
            Vector3 nextWorld = _worldPathFloorPositions[nextIdx] + lift;
            Vector3 tangent = Vector3.ProjectOnPlane(nextWorld - worldPos, Vector3.up);
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

    /// <summary>World-space goal used for the floor marker and off-screen cue (path tail cell + lift).</summary>
    private bool TryGetWorldPathDestinationWorld(out Vector3 worldPos)
    {
        worldPos = default;
        if (!pathSessionActive || grid == null || grid.grid == null)
            return false;
        if (!_worldCalibCaptured || !_worldLayoutValid)
            return false;

        Vector3 baseWorld = _worldPathFloorPositions.Count > 0
            ? _worldPathFloorPositions[_worldPathFloorPositions.Count - 1] + Vector3.up * worldArrowHeightAbovePlayer
            : (_snapPathMapPoints.Count > 0
                ? WorldArrowFloorFromSnapshotMapPoint(_snapPathMapPoints[_snapPathMapPoints.Count - 1])
                : WorldArrowFloorFromSnapshot(_lineResolvedEnd.x, _lineResolvedEnd.y));
        worldPos = baseWorld + Vector3.up * endPointMarkerHeightAboveRoute;
        return true;
    }

    /// <summary>Same camera used for distance UI when possible — head-locked HUD in XR.</summary>
    private Camera ResolveHeadHudCamera()
    {
        if (distanceUICamera != null && distanceUICamera.isActiveAndEnabled)
            return distanceUICamera;
        Transform tr = ResolveTrackingTransform();
        if (tr != null)
        {
            Camera c = tr.GetComponent<Camera>();
            if (c != null && c.isActiveAndEnabled)
                return c;
            c = tr.GetComponentInParent<Camera>();
            if (c != null && c.isActiveAndEnabled)
                return c;
        }
        return ResolveDistanceCanvasCamera();
    }

    private void HideOffScreenTurnCues()
    {
        if (_offScreenCueLeft != null)
            _offScreenCueLeft.SetActive(false);
        if (_offScreenCueRight != null)
            _offScreenCueRight.SetActive(false);
    }

    private void DestroyOffScreenTurnCueInstances()
    {
        if (_offScreenCueLeft != null)
            Destroy(_offScreenCueLeft);
        if (_offScreenCueRight != null)
            Destroy(_offScreenCueRight);
        _offScreenCueLeft = null;
        _offScreenCueRight = null;
    }

    private bool EnsureOffScreenTurnCuePair(Camera cam, GameObject prefab)
    {
        if (cam == null || prefab == null)
            return false;

        if (_offScreenCueLeft == null)
        {
            _offScreenCueLeft = Instantiate(prefab, cam.transform);
            _offScreenCueLeft.name = "OffScreenTurnCue_Left";
        }
        else if (_offScreenCueLeft.transform.parent != cam.transform)
            _offScreenCueLeft.transform.SetParent(cam.transform, false);

        if (_offScreenCueRight == null)
        {
            _offScreenCueRight = Instantiate(prefab, cam.transform);
            _offScreenCueRight.name = "OffScreenTurnCue_Right";
        }
        else if (_offScreenCueRight.transform.parent != cam.transform)
            _offScreenCueRight.transform.SetParent(cam.transform, false);

        _offScreenCueLeft.transform.localScale = offScreenTurnCueLocalScale;
        _offScreenCueRight.transform.localScale = offScreenTurnCueLocalScale;
        return true;
    }

    private void UpdateOffScreenTurnCues()
    {
        GameObject prefab = offScreenTurnCuePrefab != null ? offScreenTurnCuePrefab : worldArrowPrefab;
        if (prefab == null || !TryGetWorldPathDestinationWorld(out Vector3 destWorld))
        {
            HideOffScreenTurnCues();
            return;
        }

        Camera cam = ResolveHeadHudCamera();
        if (cam == null)
        {
            HideOffScreenTurnCues();
            return;
        }

        if (!EnsureOffScreenTurnCuePair(cam, prefab))
        {
            HideOffScreenTurnCues();
            return;
        }

        Vector3 vp = cam.WorldToViewportPoint(destWorld);
        float m = Mathf.Clamp(offScreenTurnCueViewportMargin, 0f, 0.45f);
        bool inView = vp.z > 0f
            && vp.x >= m && vp.x <= 1f - m
            && vp.y >= m && vp.y <= 1f - m;

        if (inView)
        {
            HideOffScreenTurnCues();
            return;
        }

        Vector3 flatFwd = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up);
        if (flatFwd.sqrMagnitude < 1e-8f)
            flatFwd = cam.transform.forward;
        else
            flatFwd.Normalize();

        Vector3 toDest = destWorld - cam.transform.position;
        Vector3 flatTo = Vector3.ProjectOnPlane(toDest, Vector3.up);
        if (flatTo.sqrMagnitude < 1e-8f)
        {
            HideOffScreenTurnCues();
            return;
        }
        flatTo.Normalize();

        float signedYaw = Vector3.SignedAngle(flatFwd, flatTo, Vector3.up);
        if (Mathf.Abs(signedYaw) < 0.5f)
        {
            HideOffScreenTurnCues();
            return;
        }

        bool turnRightIsShorter = signedYaw > 0f;
        GameObject show = turnRightIsShorter ? _offScreenCueRight : _offScreenCueLeft;
        GameObject hide = turnRightIsShorter ? _offScreenCueLeft : _offScreenCueRight;
        hide.SetActive(false);
        show.SetActive(true);

        float x = Mathf.Abs(offScreenTurnCueLocalX);
        show.transform.localPosition = turnRightIsShorter
            ? new Vector3(x, offScreenTurnCueLocalY, offScreenTurnCueLocalDepth)
            : new Vector3(-x, offScreenTurnCueLocalY, offScreenTurnCueLocalDepth);

        Vector3 dirWorld = turnRightIsShorter ? cam.transform.right : -cam.transform.right;
        Quaternion aimWorld =
            Quaternion.AngleAxis(worldArrowCompassYawDegrees, Vector3.up)
            * WorldArrowBaseRotation(dirWorld, worldArrowMeshForwardAxis)
            * Quaternion.Euler(worldArrowMeshPitchDegrees, worldArrowLookYawOffsetDegrees, 0f)
            * Quaternion.Euler(offScreenTurnCueRotationExtraEuler);
        show.transform.localRotation = Quaternion.Inverse(cam.transform.rotation) * aimWorld;
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

        float remainingFeet = GetRemainingRouteDistanceFeet();
        string line = remainingFeet <= Mathf.Max(0f, destinationReachedFeet)
            ? destinationReachedText
            : $"Distance: {remainingFeet:F1} ft";
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

    private float GetRemainingRouteDistanceFeet()
    {
        float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
        if (grid == null || grid.grid == null)
            return 0f;

        if (currentPath.Count == 0)
            return Vector2.Distance(currentCoordinate, endCoordinate) * toFeet;

        float totalChart = 0f;
        Vector2 previous = currentCoordinate;

        if (distanceSumUsesRawPathListOrder)
        {
            for (int i = 0; i < currentPath.Count; i++)
            {
                Node stepNode = currentPath[i];
                Vector2 current = GridCellToChartCoordinate(stepNode.x, stepNode.y);
                totalChart += Vector2.Distance(previous, current);
                previous = current;
            }
        }
        else if (_snapPathNodes.Count == currentPath.Count && currentPath.Count > 0)
        {
            for (int i = 0; i < _snapPathNodes.Count; i++)
            {
                Node stepNode = _snapPathNodes[i];
                Vector2 current = GridCellToChartCoordinate(stepNode.x, stepNode.y);
                totalChart += Vector2.Distance(previous, current);
                previous = current;
            }
        }
        else
        {
            for (int i = 0; i < currentPath.Count; i++)
            {
                Node stepNode = GetCurrentPathNodeJourneyOrder(i);
                Vector2 current = GridCellToChartCoordinate(stepNode.x, stepNode.y);
                totalChart += Vector2.Distance(previous, current);
                previous = current;
            }
        }

        return totalChart * toFeet;
    }
}
