using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

/// <summary>
/// Which local axis of the NavArrow mesh should follow the path tangent. Most FBX arrows use +Y as the tip axis; Unity LookRotation uses +Z.
/// </summary>
/// <summary>Which preset drives <see cref="PathTest.endCoordinate"/> (Navigation cycle button).</summary>
public enum RouteEndSelection
{
    A,
    B,
    HAB
}

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
    [Tooltip("XR Origin / rig root (floor height). Walking progress uses the rig's Camera child on device; assign this root for editor testing. Optional trackingTransformOverride wins.")]
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
    [Tooltip("HoloLens / XR: camera for distance TMP / canvas only. Do not use for path movement or floor anchor — assign PathTest.player / trackingTransformOverride for that.")]
    public Camera distanceUICamera;
    [Tooltip("Whole navigation panel / rig to lift so it sits above the floor (local Y offset, applied once on enable).")]
    public Transform navigationVisualRoot;
    [Tooltip("Meters added along local +Y of navigationVisualRoot.")]
    public float navigationMenuRaiseMeters = 0.08f;

    [Header("Map coordinates (feet on chart)")]
    [Tooltip("Start/end and path distance use these chart (floor-plan) coordinates. Walking in the room updates chart position from headset movement vs the map captured at Find Path — not from projecting your camera onto the floating map panel.")]
    public Vector2 mapCoordMin;
    public Vector2 mapCoordMax;
    public Vector2 startCoordinate;
    [Header("Route end presets (A / B / HAB)")]
    [Tooltip("Chart feet for route goal A (cycle button label “A”).")]
    public Vector2 aCoordinate = new Vector2(-5635f, -9960f);
    [Tooltip("Chart feet for route goal B.")]
    public Vector2 bCoordinate = new Vector2(-5615f, -9995f);
    [Tooltip("Chart feet for route goal HAB.")]
    public Vector2 habCoordinate = new Vector2(-5670f, -10060f);
    [Tooltip("Active preset on the Navigation cycle button (A → B → HAB).")]
    public RouteEndSelection routeEndSelection = RouteEndSelection.A;
    [Tooltip("Live route goal used by pathfinding; updated from the active preset above.")]
    public Vector2 endCoordinate = new Vector2(-5635f, -9960f);
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
    [Tooltip("How fast progress advances along the A* route from chart movement (nodes per second). Prevents erratic jumps from projecting feet onto a head-locked map.")]
    [Min(0.25f)]
    public float maxJourneyNodeAdvancePerSecond = 2f;
    [Header("Debug")]
    [Tooltip("Logs path start/end, node count, distance, and map-arrow rebuilds to the Unity Console after each repath.")]
    public bool logPathDiagnostics;
    [Header("Distance display")]
    [Tooltip("When remaining route distance is at or below this value (feet), the UI shows destinationReachedText instead of a numeric distance.")]
    public float destinationReachedFeet = 10f;
    [Tooltip("Shown when remaining distance ≤ destinationReachedFeet (path session active).")]
    public string destinationReachedText = "Destination reached";
    [Tooltip("Shown instead of a numeric distance when A* cannot connect start to end (e.g. blocked by obstacles, or endpoints beyond the walkable area).")]
    public string noPathAvailableText = "No Available Path";

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
    [Tooltip("Target spacing between arrows along the route polyline (feet). Shorter remaining paths spawn fewer arrows instead of packing the same count closer together.")]
    [Min(0.5f)]
    public float pathArrowSpacingFeet = 4f;
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
    [Tooltip("Defaults to worldArrowPrefab when empty. When Parent Off Screen Turn Cue To Distance Readout is on (default), cues are parented to the Distance row on the Navigation panel so they are not occluded by the map; otherwise they are parented to the headset camera.")]
    public GameObject offScreenTurnCuePrefab;
    [Tooltip("Shrink the “visible” viewport by this fraction on each side (0 = use full screen).")]
    [Range(0f, 0.45f)]
    public float offScreenTurnCueViewportMargin = 0.04f;
    [Tooltip("Meters in front of the camera (local +Z) for the cue. Only used when the cue is parented to the headset camera (see Parent Off Screen Turn Cue To Distance Readout / HUD anchor). When parented to the Navigation distance row, anchored offsets are used instead so the cue is not drawn behind the panel.")]
    public float offScreenTurnCueLocalDepth = 1.2f;
    [Tooltip("Meters left/right of view center (camera local ±X). Only used for camera-parented cues.")]
    public float offScreenTurnCueLocalX = 0.38f;
    [Tooltip("Meters up/down from view center (camera local Y). Only used for camera-parented cues.")]
    public float offScreenTurnCueLocalY = -0.06f;
    [Tooltip("Local scale applied to the cue instance.")]
    public Vector3 offScreenTurnCueLocalScale = new Vector3(0.6f, 0.6f, 0.6f);
    [Tooltip("Extra local rotation after aiming the mesh along screen left/right (degrees).")]
    public Vector3 offScreenTurnCueRotationExtraEuler = Vector3.zero;
    [Tooltip("Optional. When set, off-screen turn cues are parented here (e.g. an empty next to the Distance label) so they share the same depth/sorting as your Navigation UI instead of sitting behind the panel on the camera.")]
    public Transform offScreenTurnCueHudAnchor;
    [Tooltip("When true (default), parent turn cues to the first bound distance readout (BindDistanceReadout / Navigation sidebar) so they sit on the same UI plane as the Distance text. Falls back to the headset camera when no readout is bound. Ignored if Off Screen Turn Cue HUD Anchor is assigned.")]
    public bool parentOffScreenTurnCueToDistanceReadout = true;
    [Tooltip("When parented to a RectTransform (distance readout or HUD anchor), horizontal offset magnitude from the readout’s pivot: right cue uses +X, left cue uses −X (same |value|).")]
    public float offScreenTurnCueHudAnchoredOffsetX = 120f;
    [Tooltip("When parented to a RectTransform, vertical offset from the readout’s pivot (anchoredPosition Y).")]
    public float offScreenTurnCueHudAnchoredOffsetY = 2f;
    [Tooltip("Uniform local scale for the cue mesh when parented under UI (world-space canvas units differ from meters). Increase if the arrow is still tiny on HoloLens.")]
    public float offScreenTurnCueHudMeshUniformScale = 96f;
    [Tooltip("When parented under UI, optional MeshRenderer.sortingOrder so the cue draws above the map image on the same canvas. 0 = leave default.")]
    public int offScreenTurnCueHudMeshSortingOrder = 32000;

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
    [Tooltip("Flip the tracking transform's forward axis before computing the Orient North yaw. Use when your tracking transform / Main Camera reports its +Z axis OPPOSITE to your actual gaze (common on editor cameras parked facing the panel) so pressing Orient North while looking forward stops laying out the world route behind you.")]
    public bool invertOrientNorthFacing = false;

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
    private Vector2Int _chartGridAtLastRepath;

    /// <summary>World-space base for the red floor marker (last polyline point + arrow height). Frozen after first successful layout for <see cref="_sessionWorldGoalLockEndCell"/>; repaths only move the trail start, not this goal.</summary>
    private Vector3 _sessionWorldGoalLockBaseWorld;
    private bool _sessionWorldGoalLockValid;
    private Vector2Int _sessionWorldGoalLockEndCell;

    private Vector3 lastPlayerWorldPos;
    private Vector2 currentCoordinate;
    /// <summary>Chart + world rig pose at Find Path; chart updates from frozen-map inverse + offset (and horizontal world delta fallback).</summary>
    private Vector2 _sessionChartAtMovementCalib;
    private Vector3 _sessionTrackingWorldAtCalib;
    private Vector2 _chartOffsetFromFrozenWorld;
    private bool _sessionChartMovementValid;
    /// <summary>Floor anchor at route start (Find Path); progress is arc-length along the full session floor polyline.</summary>
    private Vector3 _sessionRouteStartFeetWorld;
    private bool _sessionRouteStartFeetValid;
    private readonly List<Node> _sessionFullSnapNodes = new List<Node>();
    private readonly List<Vector3> _sessionFullWorldFloorPolyline = new List<Vector3>();
    private bool _sessionFullRouteValid;
    /// <summary>Unclamped arc along session floor poly / poly length — negative if you stepped past the plotted start.</summary>
    private float _sessionRouteProgressT;
    /// <summary>When true, next Find Path restores chart/route progress instead of resetting to <see cref="startCoordinate"/>.</summary>
    private bool _persistNavigationProgress;
    private Vector2 _lastMapVisualChartCoord;
    /// <summary>Monotonic index along <see cref="_snapPathNodes"/> for distance + repath start (world polyline can shrink while chart start was stuck at session origin).</summary>
    private int _journeyProgressIndex;
    private float repathTimer;
    private Vector3 repathAnchorWorld;
    private Vector3 navigationRootInitialLocalPos;
    private bool navigationRootBaselineStored;
    private bool distanceUiCameraWarningLogged;
    private bool trackingTransformWarningLogged;
    private bool orientNorthCalibWarningLogged;
    private bool endPointMarkerScaleWarningLogged;
    private Transform cachedEndPointWorldMarker;
    private Vector3 cachedEndPointWorldMarkerBaseLocalScale;
    private bool mapNorthReferenceRenderersHidden;
    /// <summary>True while the Navigation floating panel is closed/disabled but a path session is still active — hide map markers/line/map arrows without stopping pathfinding HUD.</summary>
    private bool _suppressMapFaceVisualsWhileNavPanelClosed;

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
        if (!_worldCalibCaptured || CanRegenerateGridFromLiveMap())
            CaptureWorldSpatialCalibration(pathSessionActive && _sessionFullRouteValid);
        if (!_worldCalibCaptured)
        {
            if (!orientNorthCalibWarningLogged)
            {
                orientNorthCalibWarningLogged = true;
                Debug.LogWarning(
                    "PathTest: Orient North skipped — no spatial calibration. Open the navigation map or start Find Path first.");
            }
            return;
        }

        Vector3 mapN = GetMapNorthDirectionWorldHorizontal(true);
        Transform tr = ResolveTrackingTransform();
        if (tr == null)
            return;
        Vector3 rawForward = tr.forward;
        if (invertOrientNorthFacing)
            rawForward = -rawForward;
        Vector3 userF = Vector3.ProjectOnPlane(rawForward, Vector3.up);
        if (userF.sqrMagnitude < 1e-8f)
            return;
        userF.Normalize();

        Vector2 preservedChart = currentCoordinate;
        float routeTBeforeOrient = _sessionRouteProgressT;

        _sessionWorldGoalLockValid = false;
        lockedNorthYawDegrees = Vector3.SignedAngle(mapN, userF, Vector3.up);
        northOrientationLocked = true;

        // User is facing map north here: anchor the floor route at their feet and rebuild world layout in the new basis.
        _sessionRouteStartFeetWorld = GetNavigationWorldPosition();
        _sessionRouteStartFeetValid = true;

        // Keep remaining distance: full snap still referenced original path start unless we clip to current progress first.
        if (_sessionFullRouteValid && routeTBeforeOrient > 1e-4f)
            TrimSessionFullRoutePrefixBeforeReanchor(routeTBeforeOrient);

        CaptureWorldSpatialCalibration(pathSessionActive && _sessionFullRouteValid);

        if (_sessionFullRouteValid)
            RebuildSessionFullWorldFloorPolylineAndGoalLock();

        if (pathSessionActive)
        {
            currentCoordinate = preservedChart;

            if (_sessionFullRouteValid && _sessionFullWorldFloorPolyline.Count >= 2)
            {
                Vector3 feet = GetNavigationWorldPosition();
                float wTot = PolylineHorizontalLengthMeters(_sessionFullWorldFloorPolyline);
                if (wTot > 1e-8f)
                    _sessionRouteProgressT = GetHorizontalPolylineClosestArcAlongUnbounded(
                                                 _sessionFullWorldFloorPolyline, feet)
                                             / wTot;
            }

            _lastMapVisualChartCoord = currentCoordinate - Vector2.one * 1000f;
            ForceRepath();
        }
    }

    /// <summary>Clear Orient North lock; world route yaw returns to chart-only (no locked heading).</summary>
    public void ClearNorthOrientationLock()
    {
        northOrientationLocked = false;
        _sessionWorldGoalLockValid = false;
        if (pathSessionActive)
        {
            CaptureWorldSpatialCalibration(_sessionFullRouteValid);
            if (_sessionFullRouteValid)
                RebuildSessionFullWorldFloorPolylineAndGoalLock();
            ForceRepath();
        }
    }

    /// <summary>Re-run north-marker hiding after <see cref="mapNorthReference"/> is assigned at runtime (e.g. from Navigation prefab).</summary>
    public void RefreshMapNorthReferenceVisibility()
    {
        mapNorthReferenceRenderersHidden = false;
        ApplyMapNorthReferenceVisibility();
    }

    /// <summary>Chart↔world for movement integration uses calibration captured at Find Path so a head-locked map does not zero out walking deltas.</summary>
    private bool UseSessionFrozenMapSpatial() => pathSessionActive && _worldCalibCaptured;

    /// <summary>Rig root reference (mainly floor Y). Prefer <see cref="GetNavigationWorldPosition"/> for walking.</summary>
    private Transform ResolveMovementTransform()
    {
        if (trackingTransformOverride != null)
            return trackingTransformOverride;
        if (player != null)
            return player;
        return ResolveTrackingTransform();
    }

    /// <summary>
    /// World position that tracks physical walking. Uses the HMD/camera (moves on HoloLens); dragging the XR Origin in-editor
    /// still works because the camera child moves with the rig.
    /// </summary>
    private Vector3 GetNavigationWorldPosition()
    {
        if (trackingTransformOverride != null)
            return trackingTransformOverride.position;

        Transform tracking = ResolveTrackingTransform();
        Vector3 pos = tracking != null ? tracking.position : _worldCalibAnchor;
        if (player != null)
            pos.y = player.position.y;
        return pos;
    }

    private Transform ResolveTrackingTransform()
    {
        if (trackingTransformOverride != null)
            return trackingTransformOverride;

        if (player != null)
        {
            Camera childCam = player.GetComponentInChildren<Camera>(true);
            if (childCam != null && childCam.isActiveAndEnabled)
                return childCam.transform;
            if (player.GetComponent<Camera>() is Camera selfCam && selfCam.isActiveAndEnabled)
                return player;
            return player;
        }

        if (Camera.main != null && Camera.main.isActiveAndEnabled && !IsLikelyNonTrackingCamera(Camera.main))
            return Camera.main.transform;

        Camera[] cams = FindObjectsOfType<Camera>();
        Camera best = null;
        for (int i = 0; i < cams.Length; i++)
        {
            Camera c = cams[i];
            if (c == null || !c.isActiveAndEnabled || !c.gameObject.activeInHierarchy)
                continue;
            if (c.targetTexture != null)
                continue;
            if (IsLikelyNonTrackingCamera(c))
                continue;
            best = c;
            break;
        }

        if (best != null)
            return best.transform;

        if (!trackingTransformWarningLogged)
        {
            trackingTransformWarningLogged = true;
            Debug.LogWarning(
                "PathTest: No tracking transform. Assign player to the XR head/camera (not a static XR Origin root), or trackingTransformOverride.");
        }

        return null;
    }

    private static bool IsLikelyNonTrackingCamera(Camera cam)
    {
        if (cam == null)
            return true;
        Canvas canvas = cam.GetComponentInParent<Canvas>();
        return canvas != null && canvas.renderMode != RenderMode.WorldSpace;
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

        _suppressMapFaceVisualsWhileNavPanelClosed = false;
        grid.mapTransform = mapSurface;
        grid.gridAreaTransform = mapSurface;
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

    public RouteEndSelection ActiveRouteEndSelection => routeEndSelection;

    public string GetRouteEndSelectionLabel() => routeEndSelection.ToString();

    private Vector2 GetRouteEndPresetCoordinate(RouteEndSelection selection)
    {
        return selection switch
        {
            RouteEndSelection.B => bCoordinate,
            RouteEndSelection.HAB => habCoordinate,
            _ => aCoordinate
        };
    }

    /// <summary>Keeps chart goals on the authored floor-plan rectangle so grid UVs do not clamp to a black border.</summary>
    private Vector2 ClampChartCoordinateToMapBounds(Vector2 chart)
    {
        float minX = Mathf.Min(mapCoordMin.x, mapCoordMax.x);
        float maxX = Mathf.Max(mapCoordMin.x, mapCoordMax.x);
        float minY = Mathf.Min(mapCoordMin.y, mapCoordMax.y);
        float maxY = Mathf.Max(mapCoordMin.y, mapCoordMax.y);
        return new Vector2(
            Mathf.Clamp(chart.x, minX, maxX),
            Mathf.Clamp(chart.y, minY, maxY));
    }

    /// <summary>Copies the active A / B / HAB preset into <see cref="endCoordinate"/> (pathfinding uses end only).</summary>
    public void ApplyRouteEndSelectionToEndCoordinate()
    {
        endCoordinate = ClampChartCoordinateToMapBounds(GetRouteEndPresetCoordinate(routeEndSelection));
    }

    /// <summary>Push route presets from a <see cref="Navigation"/> inspector (clamped to map bounds).</summary>
    public void ApplyRoutePresetsFromNavigation(Vector2 a, Vector2 b, Vector2 hab, RouteEndSelection selection)
    {
        aCoordinate = ClampChartCoordinateToMapBounds(a);
        bCoordinate = ClampChartCoordinateToMapBounds(b);
        habCoordinate = ClampChartCoordinateToMapBounds(hab);
        routeEndSelection = selection;
        ApplyRouteEndSelectionToEndCoordinate();
    }

    /// <summary>Navigation cycle button: A → B → HAB; updates end coordinate and repaths if a session is active.</summary>
    public void CycleRouteEndSelection()
    {
        routeEndSelection = (RouteEndSelection)(((int)routeEndSelection + 1) % 3);
        ApplyRouteEndSelectionToEndCoordinate();
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
        if (ResolveTrackingTransform() != null || player != null || trackingTransformOverride != null)
            repathAnchorWorld = GetNavigationWorldPosition();
        if (!active)
        {
            _suppressMapFaceVisualsWhileNavPanelClosed = false;
            _persistNavigationProgress = HasNavigationProgressToResume();
            ClearPathSessionVisualsOnly();
        }
        else
        {
            bool resume = _persistNavigationProgress;
            _persistNavigationProgress = false;

            _suppressMapFaceVisualsWhileNavPanelClosed = false;
            TryRefreshNavigationMapBinding();
            if (ResolveTrackingTransform() != null || player != null || trackingTransformOverride != null)
                lastPlayerWorldPos = GetNavigationWorldPosition();
            if (mapStartPoint != null)
                mapStartPoint.gameObject.SetActive(true);
            if (endPoint != null)
                endPoint.gameObject.SetActive(true);

            if (!resume)
                ResetNavigationProgressState();
            else
                _lastMapVisualChartCoord = currentCoordinate;

            CaptureWorldSpatialCalibration(resume);
            ForceRepath();
        }
    }

    private bool HasNavigationProgressToResume()
    {
        if (_sessionFullRouteValid && Mathf.Abs(_sessionRouteProgressT) > 1e-4f)
            return true;
        return Vector2.Distance(currentCoordinate, startCoordinate) > 0.5f;
    }

    private void ResetNavigationProgressState()
    {
        currentCoordinate = startCoordinate;
        _sessionWorldGoalLockValid = false;
        _journeyProgressIndex = 0;
        _sessionChartMovementValid = false;
        _sessionRouteStartFeetValid = false;
        _sessionFullRouteValid = false;
        _sessionFullSnapNodes.Clear();
        _sessionFullWorldFloorPolyline.Clear();
        _sessionRouteProgressT = 0f;
        _lastMapVisualChartCoord = currentCoordinate;
    }

    /// <summary>Rebinds GridManager to the Navigation panel that is currently open so map markers match the menu (not the scene prefab pose).</summary>
    public void TryRefreshNavigationMapBinding()
    {
        Navigation.RefreshPathfindingMapBindingStatic();
    }

    private void TryBindFirstActiveNavigationInScene()
    {
        if (grid?.mapTransform != null && grid.mapTransform.GetComponentInParent<Navigation>() != null)
            return;

        Navigation[] navs = Object.FindObjectsByType<Navigation>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < navs.Length; i++)
        {
            if (navs[i] != null && navs[i].isActiveAndEnabled)
            {
                navs[i].SyncPathfindingToActiveMapPlane();
                return;
            }
        }
    }

    /// <summary>
    /// Called when the Navigation floating menu is disabled/closed (e.g. X) while a path session may still be active.
    /// Hides map-only markers and stops sampling a disabled map for grid regen; distance / world route / off-screen cues keep updating on PathTest.
    /// </summary>
    public void NotifyNavigationPanelHidden()
    {
        if (!pathSessionActive)
            return;
        _suppressMapFaceVisualsWhileNavPanelClosed = true;
        SetMapVisualsVisible(false);
        // Rebuild world-route snapshot immediately using frozen map geometry so distance + off-screen cues do not wait
        // for the next repath interval (and so layout is not stuck behind a failed live-rect WorldToGrid sample).
        ForceRepath();
    }

    /// <summary>Hides route visuals only (Stop Path). Progress, goal lock, and full route are kept for resume.</summary>
    private void ClearPathSessionVisualsOnly()
    {
        currentPath.Clear();
        _worldLayoutValid = false;
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

    /// <summary>Full reset (scene load / inactive PathTest at Start).</summary>
    private void ClearPathVisualization()
    {
        ResetNavigationProgressState();
        _worldCalibCaptured = false;
        _persistNavigationProgress = false;
        ClearPathSessionVisualsOnly();
    }

    private void OnValidate()
    {
        aCoordinate = ClampChartCoordinateToMapBounds(aCoordinate);
        bCoordinate = ClampChartCoordinateToMapBounds(bCoordinate);
        habCoordinate = ClampChartCoordinateToMapBounds(habCoordinate);
        ApplyRouteEndSelectionToEndCoordinate();
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
        ApplyRouteEndSelectionToEndCoordinate();
        TryRefreshNavigationMapBinding();
        TryBindFirstActiveNavigationInScene();
        if (grid != null && grid.grid == null && grid.mapTransform != null)
            grid.GenerateGrid();
        if (ResolveTrackingTransform() != null || player != null)
        {
            Vector3 navPos = GetNavigationWorldPosition();
            lastPlayerWorldPos = navPos;
            repathAnchorWorld = navPos;
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

        // SAFETY: if mapNorthReference points at a prefab ASSET (e.g. someone dragged NavArrow.prefab into this slot
        // in the inspector), GetComponentsInChildren walks the asset and Unity persists renderer.enabled=false to disk —
        // which silently breaks every future Instantiate of that prefab. Refuse to operate on anything that is not a scene instance.
        if (!mapNorthReference.gameObject.scene.IsValid())
        {
            Debug.LogWarning(
                $"PathTest: 'Map North Reference' is assigned to a prefab asset ('{mapNorthReference.name}'), not a scene transform. " +
                "Skipping renderer hide — otherwise Unity would persist Renderer.enabled=false back into the prefab asset (breaking every Instantiate). " +
                "Clear this field on the scene PathTest, or assign it to a scene-instance Transform (e.g. an empty under the Navigation prefab).");
            mapNorthReferenceRenderersHidden = true;
            return;
        }

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
            : (!CanRegenerateGridFromLiveMap() && _worldCalibCaptured
                ? _worldCalibTiltToHorizontal
                : Quaternion.FromToRotation(grid.GetGridPlaneNormal(), Vector3.up));
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

        Vector3 center;
        if (!CanRegenerateGridFromLiveMap() && _worldCalibCaptured)
            center = _worldCalibIsRect ? FrozenRectMapPlaneCenter() : _worldCalibCenter;
        else
            center = grid.SnapOntoVisualMapFace(grid.MapPlaneCenter);

        if (mapNorthReference != null && mapNorthReference.gameObject.activeInHierarchy)
        {
            Vector3 refOnPlane = CanRegenerateGridFromLiveMap()
                ? grid.SnapOntoVisualMapFace(mapNorthReference.position)
                : ProjectOntoFrozenMapPlane(mapNorthReference.position);
            Vector3 d = refOnPlane - center;
            return MapPlaneTangentToHorizontalWorld(d, preferCapturedTilt);
        }

        int mx = Mathf.Clamp(grid.GridWidth / 2, 0, grid.GridWidth - 1);
        int my = Mathf.Clamp(grid.GridHeight / 2, 0, grid.GridHeight - 1);
        int ny = Mathf.Min(grid.GridHeight - 1, my + 1);
        Vector3 c = MapWorldFromCellIndices(mx, my);
        Vector3 n = MapWorldFromCellIndices(mx, ny);
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
        if (pathSessionActive)
            AdvanceJourneyProgress();

        if (!pathSessionActive)
            return;

        Vector2Int chartGridNow = ChartCoordinateToGridCell(currentCoordinate);
        if (chartGridNow != _chartGridAtLastRepath)
        {
            _chartGridAtLastRepath = chartGridNow;
            repathAnchorWorld = GetNavigationWorldPosition();
            ForceRepath();
            repathTimer = 0f;
        }

        repathTimer += Time.deltaTime;
        bool intervalElapsed = repathTimer >= Mathf.Max(0.05f, repathIntervalSeconds);
        if (intervalElapsed)
        {
            repathTimer = 0f;
            if (ResolveTrackingTransform() != null || player != null || trackingTransformOverride != null)
            {
                Vector3 navPos = GetNavigationWorldPosition();
                float moved = Vector3.Distance(navPos, repathAnchorWorld);
                if (moved >= repathMinPlayerMoveMeters)
                {
                    repathAnchorWorld = navPos;
                    ForceRepath();
                }
            }
            else
            {
                ForceRepath();
            }
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

        if (!IsMapFaceShownForVisuals())
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
        PlaceEndpointMarkers();
        RefreshMapRouteVisualsFromCurrentChartPosition();
        UpdateEndPointWorldMarker();
        UpdateOffScreenTurnCues();
    }

    private bool IsMapSurfaceVisible()
    {
        if (grid == null || grid.ActiveMapTransform == null)
            return false;
        return grid.ActiveMapTransform.gameObject.activeInHierarchy;
    }

    private bool IsMapFaceShownForVisuals()
    {
        return IsMapSurfaceVisible() && !_suppressMapFaceVisualsWhileNavPanelClosed;
    }

    private bool CanRegenerateGridFromLiveMap()
    {
        return grid != null && grid.ActiveMapTransform != null && grid.ActiveMapTransform.gameObject.activeInHierarchy;
    }

    /// <summary>Map markers, arrows, and line — always on the live map face when the panel is visible.</summary>
    private Vector3 MapWorldFromCellIndices(int xi, int yi)
    {
        if (grid == null)
            return Vector3.zero;
        if (!CanRegenerateGridFromLiveMap() && _worldCalibCaptured)
        {
            if (_worldCalibIsRect)
                return grid.GridToWorldUsingRectCorners(xi, yi, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            return GridToWorldSnapshot(xi, yi);
        }
        return grid.GridToWorld(xi, yi);
    }

    /// <summary>Chart coordinate → world for movement deltas. Always frozen during an active session so a head-locked map does not zero out walking.</summary>
    private Vector3 MapWorldFromCellIndicesForChartMovement(int xi, int yi)
    {
        if (grid == null)
            return Vector3.zero;
        if (UseSessionFrozenMapSpatial())
        {
            if (_worldCalibIsRect)
                return grid.GridToWorldUsingRectCorners(xi, yi, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            return GridToWorldSnapshot(xi, yi);
        }
        if (!CanRegenerateGridFromLiveMap() && _worldCalibCaptured)
        {
            if (_worldCalibIsRect)
                return grid.GridToWorldUsingRectCorners(xi, yi, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            return GridToWorldSnapshot(xi, yi);
        }
        return grid.GridToWorld(xi, yi);
    }

    private Vector3 MapCellWorldSnapped(int xi, int yi)
    {
        if (grid == null)
            return Vector3.zero;
        if (CanRegenerateGridFromLiveMap())
            return grid.SnapOntoVisualMapFace(grid.GridToWorld(xi, yi));
        return MapWorldFromCellIndices(xi, yi);
    }

    private Vector2Int WorldToGridForSession(Vector3 worldPos)
    {
        if (grid == null)
            return Vector2Int.zero;
        if (CanRegenerateGridFromLiveMap())
            return grid.WorldToGrid(worldPos);
        if (_worldCalibCaptured)
        {
            if (_worldCalibIsRect)
                return grid.WorldToGridFromCorners(worldPos, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            return WorldToGridAxisSnapshot(worldPos);
        }
        return grid.WorldToGrid(worldPos);
    }

    /// <summary>
    /// Same cell mapping as <see cref="GridManager"/> axis layout, using the last captured map pose (Navigation panel disabled).
    /// </summary>
    private Vector2Int WorldToGridAxisSnapshot(Vector3 worldPos)
    {
        Vector3 local = worldPos - _worldCalibCenter;
        float cellW = _worldCalibMapW / Mathf.Max(1, _worldCalibGw);
        float cellH = _worldCalibMapH / Mathf.Max(1, _worldCalibGh);
        int gx = Mathf.Clamp(
            Mathf.FloorToInt((Vector3.Dot(local, _worldCalibAxisX) + _worldCalibMapW * 0.5f) / Mathf.Max(1e-8f, cellW)),
            0,
            _worldCalibGw - 1);
        int gy = Mathf.Clamp(
            Mathf.FloorToInt((Vector3.Dot(local, _worldCalibAxisY) + _worldCalibMapH * 0.5f) / Mathf.Max(1e-8f, cellH)),
            0,
            _worldCalibGh - 1);
        return new Vector2Int(gx, gy);
    }

    private Vector3 FrozenRectMapPlaneCenter()
    {
        Vector3 c0 = _worldCalibCorner0;
        Vector3 ex = _worldCalibCorner3 - c0;
        Vector3 ey = _worldCalibCorner1 - c0;
        return c0 + 0.5f * ex + 0.5f * ey;
    }

    private Vector3 ProjectOntoFrozenMapPlane(Vector3 worldPos)
    {
        if (!_worldCalibCaptured)
            return worldPos;

        Vector3 planePoint;
        Vector3 n = GetFrozenMapPlaneNormal(out planePoint);
        if (n.sqrMagnitude < 1e-12f)
            return worldPos;
        return worldPos - Vector3.Dot(worldPos - planePoint, n) * n;
    }

    private Vector3 GetFrozenMapPlaneNormal(out Vector3 planePoint)
    {
        if (_worldCalibIsRect)
        {
            Vector3 c0 = _worldCalibCorner0;
            Vector3 ex = _worldCalibCorner3 - c0;
            Vector3 ey = _worldCalibCorner1 - c0;
            planePoint = c0;
            return Vector3.Cross(ex, ey).normalized;
        }

        planePoint = _worldCalibCenter;
        return Vector3.Cross(_worldCalibAxisX, _worldCalibAxisY).normalized;
    }

    private Vector3 MapFaceOutForSession()
    {
        if (CanRegenerateGridFromLiveMap())
            return grid.MapFaceOut;
        if (_worldCalibCaptured && _worldCalibIsRect)
        {
            Vector3 c0 = _worldCalibCorner0;
            Vector3 n = Vector3.Cross(_worldCalibCorner3 - c0, _worldCalibCorner1 - c0);
            if (n.sqrMagnitude < 1e-12f)
                return Vector3.forward;
            return n.normalized;
        }
        return grid != null ? grid.MapFaceOut : Vector3.forward;
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
        if (grid == null || (ResolveTrackingTransform() == null && player == null && trackingTransformOverride == null))
            return;

        Vector3 worldPos = GetNavigationWorldPosition();
        Vector3 delta = worldPos - lastPlayerWorldPos;
        lastPlayerWorldPos = worldPos;

        // Same horizontal feet→goal model as the world floor polyline.
        if (pathSessionActive && _sessionWorldGoalLockValid && _sessionRouteStartFeetValid
            && SyncChartPositionFromFeetProgressAlongRoute())
            return;

        if (UseSessionFrozenMapSpatial() && _sessionChartMovementValid)
        {
            if (TryGetChartFromFrozenWorldPosition(worldPos, out Vector2 chartOnFrozen))
                currentCoordinate = chartOnFrozen + _chartOffsetFromFrozenWorld;
            else
            {
                Vector3 worldMotion = worldPos - _sessionTrackingWorldAtCalib;
                Vector3 horizontalMotion = Vector3.ProjectOnPlane(worldMotion, Vector3.up);
                currentCoordinate = ApplyWorldMotionToChart(_sessionChartAtMovementCalib, horizontalMotion);
            }

            return;
        }

        if (delta.sqrMagnitude <= Mathf.Epsilon)
            return;

        Vector3 horizontalDelta = Vector3.ProjectOnPlane(delta, Vector3.up);
        currentCoordinate = ApplyWorldMotionToChart(currentCoordinate, horizontalDelta);
    }

    /// <summary>
    /// Advances <see cref="currentCoordinate"/> by arc-length along the full session floor route (same parameter for map + world).
    /// </summary>
    private bool SyncChartPositionFromFeetProgressAlongRoute()
    {
        if (!_sessionFullRouteValid || _sessionFullSnapNodes.Count < 2 || _sessionFullWorldFloorPolyline.Count < 2)
            return false;

        Vector3 feet = GetNavigationWorldPosition();
        float wTot = PolylineHorizontalLengthMeters(_sessionFullWorldFloorPolyline);
        if (wTot < 1e-8f)
            return false;

        float arc = GetHorizontalPolylineClosestArcAlongUnbounded(_sessionFullWorldFloorPolyline, feet);
        _sessionRouteProgressT = arc / wTot;

        float chartLen = TotalChartPolylineUnits(_sessionFullSnapNodes);
        currentCoordinate = SampleChartAtDistanceAlongNodes(_sessionFullSnapNodes, _sessionRouteProgressT * chartLen);
        return true;
    }

    private Vector3 GetLockedWorldGoalFloorPosition()
    {
        Vector3 goal = _sessionWorldGoalLockBaseWorld - Vector3.up * worldArrowHeightAbovePlayer;
        if (player != null)
            goal.y = player.position.y;
        return goal;
    }

    private void TryCaptureSessionFullRoute(IReadOnlyList<Vector3> mapPointsForSnap)
    {
        if (_sessionFullRouteValid || _snapPathNodes.Count < 5 || !_sessionWorldGoalLockValid)
            return;

        Vector2Int sessionStart = ChartCoordinateToGridCell(startCoordinate);
        if ((sessionStart - _lineResolvedStart).sqrMagnitude > 9)
            return;

        if (!_sessionRouteStartFeetValid)
        {
            _sessionRouteStartFeetWorld = GetNavigationWorldPosition();
            _sessionRouteStartFeetValid = true;
        }

        _sessionFullSnapNodes.Clear();
        for (int i = 0; i < _snapPathNodes.Count; i++)
            _sessionFullSnapNodes.Add(_snapPathNodes[i]);

        RebuildSessionFullWorldFloorPolylineAndGoalLock(mapPointsForSnap);
        _sessionRouteProgressT = 0f;
    }

    /// <summary>
    /// Lays the full chart A* route on the floor using calibrated chart north (Orient North) from route-start feet to the goal.
    /// </summary>
    private void RebuildSessionFullWorldFloorPolylineAndGoalLock(IReadOnlyList<Vector3> mapPointsForSnap = null)
    {
        if (_sessionFullSnapNodes.Count < 2)
            return;

        var mapPts = new List<Vector3>(_sessionFullSnapNodes.Count);
        if (mapPointsForSnap != null && mapPointsForSnap.Count == _sessionFullSnapNodes.Count)
        {
            for (int i = 0; i < mapPointsForSnap.Count; i++)
                mapPts.Add(mapPointsForSnap[i]);
        }
        else
        {
            for (int i = 0; i < _sessionFullSnapNodes.Count; i++)
            {
                Node n = _sessionFullSnapNodes[i];
                mapPts.Add(MapCellWorldSnapped(n.x, n.y));
            }
        }

        Vector3 w0 = _sessionRouteStartFeetValid
            ? _sessionRouteStartFeetWorld
            : GetNavigationWorldPosition();
        Vector3 w1 = ComputeWorldGoalFloorFromSnapLayout(w0, _sessionFullSnapNodes, mapPts);

        BuildStretchedWorldFloorPolyline(_sessionFullSnapNodes, mapPts, w0, w1, _sessionFullWorldFloorPolyline);
        _sessionFullRouteValid = _sessionFullWorldFloorPolyline.Count >= 2;

        if (_sessionFullRouteValid && worldRouteGoalWorld == null)
            EstablishSessionGoalLockFromFullPolyline();
    }

    private void EstablishSessionGoalLockFromFullPolyline()
    {
        if (_sessionFullWorldFloorPolyline.Count < 1 || _sessionFullSnapNodes.Count < 1)
            return;

        Vector3 lockFloor = _sessionFullWorldFloorPolyline[_sessionFullWorldFloorPolyline.Count - 1];
        _sessionWorldGoalLockBaseWorld = lockFloor + Vector3.up * worldArrowHeightAbovePlayer;
        Node tail = _sessionFullSnapNodes[_sessionFullSnapNodes.Count - 1];
        _sessionWorldGoalLockEndCell = new Vector2Int(tail.x, tail.y);
        _sessionWorldGoalLockValid = true;
    }

    private Vector3 ComputeWorldGoalFloorFromSnapLayout(
        Vector3 w0, IReadOnlyList<Node> nodes, IReadOnlyList<Vector3> mapPoints)
    {
        if (worldRouteGoalWorld != null)
        {
            Vector3 w1 = worldRouteGoalWorld.position;
            w1.y = w0.y;
            return w1;
        }

        var rawFlat = new List<Vector3>();
        BuildRawFlatForWorldFloor(nodes, mapPoints, rawFlat);
        if (rawFlat.Count == 0)
            return w0;

        float extraYaw = worldFloorUsesGridLayout
            ? (northOrientationLocked ? 0f : worldPathYawOffsetDegrees)
            : worldPathYawOffsetDegrees + _worldCalibNorthYawDeg;
        if (flipWorldFloorDefaultDirection && !northOrientationLocked)
            extraYaw += 180f;

        return w0 + Quaternion.Euler(0f, extraYaw, 0f) * rawFlat[rawFlat.Count - 1];
    }

    private void BuildRawFlatForWorldFloor(
        IReadOnlyList<Node> nodes, IReadOnlyList<Vector3> mapPoints, List<Vector3> rawFlat)
    {
        rawFlat.Clear();
        if (nodes == null || nodes.Count < 1 || mapPoints == null || mapPoints.Count < 1)
            return;

        float spaceScale = Mathf.Max(0.001f, worldPathUniformSpaceScale);
        if (worldFloorUsesGridLayout)
        {
            Vector3 northDir = GetCalibratedChartNorthWorldHorizontal();
            if (northDir.sqrMagnitude < 1e-8f) northDir = Vector3.forward;
            else northDir.Normalize();
            Vector3 eastDir = Vector3.Cross(Vector3.up, northDir);
            if (eastDir.sqrMagnitude < 1e-8f) eastDir = Vector3.right;
            else eastDir.Normalize();

            Node startNode = nodes[0];
            float cellW = Mathf.Max(1e-6f, grid.CellWidth);
            float cellH = Mathf.Max(1e-6f, grid.CellHeight);
            for (int i = 0; i < nodes.Count; i++)
            {
                Node n = nodes[i];
                float gx = (n.x - startNode.x) * cellW * spaceScale;
                float gy = (n.y - startNode.y) * cellH * spaceScale;
                rawFlat.Add(eastDir * gx + northDir * gy);
            }
        }
        else
        {
            for (int i = 0; i < mapPoints.Count; i++)
            {
                Vector3 d = mapPoints[i] - mapPoints[0];
                Vector3 flat = Vector3.ProjectOnPlane(_worldCalibTiltToHorizontal * d * spaceScale, Vector3.up);
                rawFlat.Add(flat);
            }
        }
    }

    private void ApplySessionRouteProgressTrimmedVisuals()
    {
        if (!_sessionFullRouteValid)
            return;

        _worldPathFloorPositions.Clear();
        float wTot = PolylineHorizontalLengthMeters(_sessionFullWorldFloorPolyline);
        AppendFloorPolylineFromArcAlongMeters(_sessionFullWorldFloorPolyline, _sessionRouteProgressT * wTot,
            _worldPathFloorPositions);

        int startIdx = GetSnapNodeIndexAtRouteProgress(_sessionFullSnapNodes, _sessionRouteProgressT);
        _snapPathNodes.Clear();
        _snapPathMapPoints.Clear();
        for (int i = startIdx; i < _sessionFullSnapNodes.Count; i++)
        {
            Node n = _sessionFullSnapNodes[i];
            _snapPathNodes.Add(n);
            _snapPathMapPoints.Add(MapCellWorldSnapped(n.x, n.y));
        }

        if (_snapPathNodes.Count > 0)
        {
            _snapStartX = _snapPathNodes[0].x;
            _snapStartY = _snapPathNodes[0].y;
            _snapMapStartWorld = _snapPathMapPoints[0];
        }

        _worldLayoutValid = _worldPathFloorPositions.Count >= 1;
    }

    private int GetSnapNodeIndexAtRouteProgress(IReadOnlyList<Node> nodes, float worldProgressRatio)
    {
        if (nodes == null || nodes.Count == 0)
            return 0;

        float chartLen = TotalChartPolylineUnits(nodes);
        if (chartLen < 1e-8f)
            return 0;

        Vector2 chart = SampleChartAtDistanceAlongNodes(nodes, worldProgressRatio * chartLen);
        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector2 c = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            float d = (c - chart).sqrMagnitude;
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }

        return best;
    }

    /// <summary>Horizontal chart length of snapped node polyline (map/chart units).</summary>
    private float TotalChartPolylineUnits(IReadOnlyList<Node> nodes)
    {
        if (nodes == null || nodes.Count < 2)
            return 0f;

        float s = 0f;
        for (int i = 1; i < nodes.Count; i++)
        {
            Vector2 a = GridCellToChartCoordinate(nodes[i - 1].x, nodes[i - 1].y);
            Vector2 b = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            s += Vector2.Distance(a, b);
        }

        return s;
    }

    /// <summary>Distance along chart edges of <paramref name="nodes"/> — negative extrapolates before node 0, past end extrapolates beyond last node.</summary>
    private Vector2 SampleChartAtDistanceAlongNodes(IReadOnlyList<Node> nodes, float distanceFromStartAlongPolyline)
    {
        if (nodes == null || nodes.Count == 0)
            return Vector2.zero;
        if (nodes.Count == 1)
            return GridCellToChartCoordinate(nodes[0].x, nodes[0].y);

        Vector2 first = GridCellToChartCoordinate(nodes[0].x, nodes[0].y);
        Vector2 second = GridCellToChartCoordinate(nodes[1].x, nodes[1].y);
        Vector2 firstSeg = second - first;
        float seg0Len = firstSeg.magnitude;

        if (distanceFromStartAlongPolyline <= 0f)
        {
            if (seg0Len < 1e-8f)
                return first;
            return first + (firstSeg / seg0Len) * distanceFromStartAlongPolyline;
        }

        float acc = 0f;
        Vector2 prev = first;
        for (int i = 1; i < nodes.Count; i++)
        {
            Vector2 cur = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            float seg = Vector2.Distance(prev, cur);
            if (acc + seg >= distanceFromStartAlongPolyline - 1e-6f)
            {
                float u = seg > 1e-8f ? (distanceFromStartAlongPolyline - acc) / seg : 0f;
                return Vector2.Lerp(prev, cur, Mathf.Clamp01(u));
            }

            acc += seg;
            prev = cur;
        }

        Vector2 cEnd = GridCellToChartCoordinate(nodes[nodes.Count - 1].x, nodes[nodes.Count - 1].y);
        Vector2 cPrevEnd = GridCellToChartCoordinate(nodes[nodes.Count - 2].x, nodes[nodes.Count - 2].y);
        Vector2 outbound = cEnd - cPrevEnd;
        float outLen = outbound.magnitude;
        if (outLen < 1e-8f)
            return cEnd;
        return cEnd + (outbound / outLen) * (distanceFromStartAlongPolyline - acc);
    }

    /// <summary>
    /// Drops path-prefix nodes already traveled (by chart route progress). Used when Orient North re-anchors world feet —
    /// otherwise rebuild still used the original chart start→goal span and snapped distance back to full length (~89 ft).
    /// </summary>
    private void TrimSessionFullRoutePrefixBeforeReanchor(float tPreserveWorldRatio)
    {
        if (!_sessionFullRouteValid || _sessionFullSnapNodes.Count < 2)
            return;

        float trimRatio = Mathf.Max(0f, tPreserveWorldRatio);
        if (trimRatio < 1e-4f)
            return;

        int startIdx = GetSnapNodeIndexAtRouteProgress(_sessionFullSnapNodes, trimRatio);
        if (startIdx <= 0)
            return;

        if (_sessionFullSnapNodes.Count - startIdx < 2)
            return;

        _sessionFullSnapNodes.RemoveRange(0, startIdx);
    }

    /// <summary>Closest arc length from first floor point along the polyline to <paramref name="worldPoint"/> (XZ). First segment can extend backward; last can extend past the end.</summary>
    private static float GetHorizontalPolylineClosestArcAlongUnbounded(
        IReadOnlyList<Vector3> polyline, Vector3 worldPoint)
    {
        if (polyline == null || polyline.Count < 2)
            return 0f;

        float y = worldPoint.y;
        float bestArc = 0f;
        float bestDistSq = float.MaxValue;
        float walked = 0f;
        int segCount = polyline.Count - 1;

        for (int i = 0; i < segCount; i++)
        {
            Vector3 a = polyline[i];
            Vector3 b = polyline[i + 1];
            a.y = y;
            b.y = y;
            Vector3 ab = b - a;
            float segLenSq = ab.x * ab.x + ab.z * ab.z;
            float segLen = Mathf.Sqrt(segLenSq);
            float tAlong;
            if (segLen < 1e-8f)
            {
                float dSq =
                    (worldPoint.x - a.x) * (worldPoint.x - a.x)
                    + (worldPoint.z - a.z) * (worldPoint.z - a.z);
                if (dSq < bestDistSq)
                {
                    bestDistSq = dSq;
                    bestArc = walked;
                }

                walked += segLen;
                continue;
            }

            tAlong =
                Vector2.Dot(new Vector2(worldPoint.x - a.x, worldPoint.z - a.z),
                    new Vector2(ab.x, ab.z)) / segLenSq;

            bool firstSeg = i == 0;
            bool lastSeg = i == segCount - 1;
            if (firstSeg && lastSeg)
            {
                // Single segment polyline — allow extrapolation either way.
            }
            else if (firstSeg)
                tAlong = Mathf.Min(1f, tAlong);
            else if (lastSeg)
                tAlong = Mathf.Max(0f, tAlong);
            else
                tAlong = Mathf.Clamp01(tAlong);

            Vector3 closest = a + ab * tAlong;
            float dSqClosest =
                (worldPoint.x - closest.x) * (worldPoint.x - closest.x)
                + (worldPoint.z - closest.z) * (worldPoint.z - closest.z);

            if (dSqClosest < bestDistSq)
            {
                bestDistSq = dSqClosest;
                bestArc = walked + tAlong * segLen;
            }

            walked += segLen;
        }

        return bestArc;
    }

    /// <summary>Remainder of floor poly from arc length <paramref name="arcAlongMeters"/> (negative extrapolates before first point).</summary>
    private static void AppendFloorPolylineFromArcAlongMeters(IReadOnlyList<Vector3> source, float arcAlongMeters,
        List<Vector3> dest)
    {
        dest.Clear();
        if (source == null || source.Count == 0)
            return;
        if (source.Count == 1)
        {
            dest.Add(source[0]);
            return;
        }

        float totalLen = PolylineHorizontalLengthMeters(source);

        float yRef = source[0].y;
        if (arcAlongMeters < -1e-5f)
        {
            Vector3 p0 = source[0];
            Vector3 p1 = source[1];
            p0.y = yRef;
            p1.y = yRef;
            Vector3 ab = p1 - p0;
            ab.y = 0f;
            float sl = ab.magnitude;
            if (sl < 1e-6f)
                dest.Add(source[0]);
            else
            {
                Vector3 q = source[0] + (ab / sl) * arcAlongMeters;
                q.y = source[0].y;
                dest.Add(q);
            }

            for (int i = 0; i < source.Count; i++)
                dest.Add(source[i]);

            return;
        }

        if (arcAlongMeters > totalLen + 1e-5f)
        {
            Vector3 a = source[source.Count - 2];
            Vector3 b = source[source.Count - 1];
            a.y = yRef;
            b.y = yRef;
            Vector3 ab = b - a;
            ab.y = 0f;
            float sl = ab.magnitude;
            if (sl < 1e-6f)
            {
                dest.Add(source[source.Count - 1]);
                return;
            }

            Vector3 q = source[source.Count - 1] + (ab / sl) * (arcAlongMeters - totalLen);
            q.y = source[source.Count - 1].y;
            dest.Add(q);
            return;
        }

        float targetLen = Mathf.Clamp(arcAlongMeters, 0f, totalLen);
        float walkedSeg = 0f;
        for (int i = 0; i < source.Count - 1; i++)
        {
            Vector3 a = source[i];
            Vector3 b = source[i + 1];
            Vector3 d = b - a;
            d.y = 0f;
            float segLen = d.magnitude;
            if (segLen < 1e-8f)
                continue;

            if (walkedSeg + segLen >= targetLen - 1e-6f)
            {
                float u = Mathf.Clamp01((targetLen - walkedSeg) / segLen);
                dest.Add(Vector3.Lerp(a, b, u));
                for (int j = i + 1; j < source.Count; j++)
                    dest.Add(source[j]);
                return;
            }

            walkedSeg += segLen;
        }

        dest.Add(source[source.Count - 1]);
    }

    private static void AppendPolylineFromArcLength(
        IReadOnlyList<Vector3> source, float t01, List<Vector3> dest)
    {
        dest.Clear();
        if (source == null || source.Count == 0)
            return;
        if (source.Count == 1 || t01 <= 1e-6f)
        {
            for (int i = 0; i < source.Count; i++)
                dest.Add(source[i]);
            return;
        }

        t01 = Mathf.Clamp01(t01);
        float totalLen = PolylineHorizontalLengthMeters(source);
        if (totalLen < 1e-8f)
        {
            dest.Add(source[source.Count - 1]);
            return;
        }

        float targetLen = t01 * totalLen;
        float walked = 0f;
        for (int i = 0; i < source.Count - 1; i++)
        {
            Vector3 a = source[i];
            Vector3 b = source[i + 1];
            Vector3 d = b - a;
            d.y = 0f;
            float segLen = d.magnitude;
            if (segLen < 1e-8f)
                continue;

            if (walked + segLen >= targetLen - 1e-6f)
            {
                float u = Mathf.Clamp01((targetLen - walked) / segLen);
                dest.Add(Vector3.Lerp(a, b, u));
                for (int j = i + 1; j < source.Count; j++)
                    dest.Add(source[j]);
                return;
            }

            walked += segLen;
        }

        dest.Add(source[source.Count - 1]);
    }

    private void BuildStretchedWorldFloorPolyline(
        IReadOnlyList<Node> nodes,
        IReadOnlyList<Vector3> mapPoints,
        Vector3 w0,
        Vector3 w1,
        List<Vector3> dest)
    {
        dest.Clear();
        if (nodes == null || nodes.Count < 1 || mapPoints == null || mapPoints.Count < 1)
            return;

        w1.y = w0.y;
        var rawFlat = new List<Vector3>();
        BuildRawFlatForWorldFloor(nodes, mapPoints, rawFlat);

        Vector3 target = Vector3.ProjectOnPlane(w1 - w0, Vector3.up);
        Vector3 source = rawFlat[rawFlat.Count - 1];
        if (source.sqrMagnitude < 1e-14f)
        {
            for (int i = 0; i < rawFlat.Count; i++)
                dest.Add(w0);
            return;
        }

        float stretch = target.magnitude / source.magnitude;
        float spin = Vector3.SignedAngle(source, target, Vector3.up);
        Quaternion spinQ = Quaternion.AngleAxis(spin, Vector3.up);
        for (int i = 0; i < rawFlat.Count; i++)
            dest.Add(w0 + spinQ * (rawFlat[i] * stretch));
    }

    private void RefreshMapRouteVisualsFromCurrentChartPosition()
    {
        if (!pathSessionActive || currentPath.Count < 2 || !IsMapFaceShownForVisuals())
            return;

        if ((currentCoordinate - _lastMapVisualChartCoord).sqrMagnitude < 0.01f)
            return;

        _lastMapVisualChartCoord = currentCoordinate;
        RebuildMapArrows();
        RebuildWorldArrows();
        if (pathLine != null && showMapPathLine)
            DrawPathLine();
    }

    private void RefreshSessionChartMovementOrigin()
    {
        _sessionChartMovementValid = false;
        if (!UseSessionFrozenMapSpatial())
            return;

        if (ResolveTrackingTransform() == null && player == null && trackingTransformOverride == null)
            return;

        _sessionChartAtMovementCalib = currentCoordinate;
        Vector3 navPos = GetNavigationWorldPosition();
        _sessionTrackingWorldAtCalib = navPos;
        if (TryGetChartFromFrozenWorldPosition(navPos, out Vector2 chartOnFrozen))
            _chartOffsetFromFrozenWorld = currentCoordinate - chartOnFrozen;
        else
            _chartOffsetFromFrozenWorld = Vector2.zero;
        _sessionChartMovementValid = true;
    }

    private bool TryGetChartFromFrozenWorldPosition(Vector3 worldPos, out Vector2 chart)
    {
        chart = Vector2.zero;
        if (!_worldCalibCaptured || grid == null)
            return false;

        Vector3 onPlane = ProjectOntoFrozenMapPlane(worldPos);
        if (_worldCalibIsRect)
        {
            if (!grid.TryWorldToFractionalGridFromCorners(
                    onPlane, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3, out float gx, out float gy))
                return false;
            chart = FractionalGridToChartCoordinate(gx, gy);
            return true;
        }

        Vector3 local = onPlane - _worldCalibCenter;
        float cw = _worldCalibMapW / Mathf.Max(1, _worldCalibGw);
        float ch = _worldCalibMapH / Mathf.Max(1, _worldCalibGh);
        float gxAxis = (Vector3.Dot(local, _worldCalibAxisX) + _worldCalibMapW * 0.5f) / Mathf.Max(1e-8f, cw) - 0.5f;
        float gyAxis = (Vector3.Dot(local, _worldCalibAxisY) + _worldCalibMapH * 0.5f) / Mathf.Max(1e-8f, ch) - 0.5f;
        chart = FractionalGridToChartCoordinate(gxAxis, gyAxis);
        return true;
    }

    private Vector3 GetLiveMapSurfaceWorldForChartCoordinate(Vector2 chart)
    {
        if (grid == null)
            return Vector3.zero;

        ChartCoordinateToFractionalGrid(chart, out float gx, out float gy);
        Vector3 onMap = CanRegenerateGridFromLiveMap()
            ? grid.GridToWorldFractional(gx, gy)
            : MapWorldFromFractionalGridForChartMovement(gx, gy);
        return CanRegenerateGridFromLiveMap() ? grid.SnapOntoVisualMapFace(onMap) : onMap;
    }

    /// <summary>Maps a world-space displacement into chart coordinates using the frozen map basis at <paramref name="originChart"/>.</summary>
    private Vector2 ApplyWorldMotionToChart(Vector2 originChart, Vector3 worldMotion)
    {
        if (worldMotion.sqrMagnitude <= Mathf.Epsilon)
            return originChart;

        const float chartProbeFeet = 1f;
        Vector3 originWorld = CoordinateToWorldContinuous(originChart);
        Vector3 axisChartX = CoordinateToWorldContinuous(originChart + new Vector2(chartProbeFeet, 0f)) - originWorld;
        Vector3 axisChartY = CoordinateToWorldContinuous(originChart + new Vector2(0f, chartProbeFeet)) - originWorld;

        Vector2 result = originChart;
        if (axisChartX.sqrMagnitude > 1e-10f)
            result.x += Vector3.Dot(worldMotion, axisChartX) * chartProbeFeet / axisChartX.sqrMagnitude;
        if (axisChartY.sqrMagnitude > 1e-10f)
            result.y += Vector3.Dot(worldMotion, axisChartY) * chartProbeFeet / axisChartY.sqrMagnitude;
        return result;
    }

    private int FindClosestJourneyIndex(Vector2 chartProbe)
    {
        if (_snapPathNodes.Count == 0)
            return 0;
        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < _snapPathNodes.Count; i++)
        {
            Vector2 c = GridCellToChartCoordinate(_snapPathNodes[i].x, _snapPathNodes[i].y);
            float d = (c - chartProbe).sqrMagnitude;
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        return best;
    }

    /// <summary>Advance along the route from chart movement (capped rate so progress cannot teleport to the goal).</summary>
    private void AdvanceJourneyProgress()
    {
        if (_snapPathNodes.Count < 2)
            return;

        int closest = FindClosestJourneyIndex(currentCoordinate);
        if (closest <= _journeyProgressIndex)
            return;

        float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
        Vector2 progressChart = GridCellToChartCoordinate(
            _snapPathNodes[_journeyProgressIndex].x, _snapPathNodes[_journeyProgressIndex].y);
        float chartAheadFt = Vector2.Distance(progressChart, currentCoordinate) * toFeet;

        int maxByRate = Mathf.Max(1, Mathf.CeilToInt(maxJourneyNodeAdvancePerSecond * Time.deltaTime));
        int maxByWalk = chartAheadFt >= pathArrowSpacingFeet * 0.5f
            ? Mathf.Max(maxByRate, Mathf.CeilToInt(chartAheadFt / Mathf.Max(0.5f, pathArrowSpacingFeet)))
            : maxByRate;
        _journeyProgressIndex = Mathf.Min(_journeyProgressIndex + maxByWalk, closest);
    }

    private bool IsJourneyNearEnd()
    {
        if (_snapPathNodes.Count < 2)
            return false;
        return _journeyProgressIndex >= _snapPathNodes.Count - 2;
    }

    private Vector3 GetWorldPathFloorAnchor(Transform trackingOverride)
    {
        if (trackingOverride != null)
        {
            Vector3 a = trackingOverride.position;
            if (player != null)
                a.y = player.position.y;
            else if (worldArrowParent != null)
                a.y = worldArrowParent.position.y;
            return a;
        }

        return GetNavigationWorldPosition();
    }

    /// <summary>
    /// Map start/end in grid cells. Start uses <see cref="currentCoordinate"/> (chart). World floor visuals anchor at each repath (see snapshot), not every frame from the live map pose.
    /// </summary>
    private Vector2Int ResolveWalkableGridCell(Vector2 chartCoordinate, bool isGoal)
    {
        Vector2Int cell = ChartCoordinateToGridCell(ClampChartCoordinateToMapBounds(chartCoordinate));
        int radius = nearestWalkableSearchRadius;
        Vector2Int resolved = grid.FindNearestWalkable(cell, radius);
        if (!grid.IsWalkable(resolved))
        {
            int expanded = isGoal ? Mathf.Max(radius, 50) : Mathf.Max(radius, 24);
            resolved = grid.FindNearestWalkable(cell, expanded);
        }

        return resolved;
    }

    private void ResolveStartEndGrid(out Vector2Int resolvedStart, out Vector2Int resolvedEnd)
    {
        resolvedStart = ResolveWalkableGridCell(currentCoordinate, isGoal: false);
        resolvedEnd = ResolveWalkableGridCell(endCoordinate, isGoal: true);
        _lineResolvedStart = resolvedStart;
        _lineResolvedEnd = resolvedEnd;
    }

    private void PlaceEndpointMarkers()
    {
        Vector3 lift = MapFaceLift(markerHeightOffset);

        if (mapStartPoint != null)
        {
            Vector3 p = GetLiveMapSurfaceWorldForChartCoordinate(currentCoordinate);
            mapStartPoint.position = p + lift;
        }

        if (endPoint != null)
        {
            Vector3 p = GetLiveMapSurfaceWorldForChartCoordinate(endCoordinate);
            endPoint.position = p + lift;
        }
    }

    private void UpdateEndPointWorldMarker()
    {
        if (endPointWorldMarker == null)
            return;
        if (!pathSessionActive || !_worldCalibCaptured)
        {
            endPointWorldMarker.gameObject.SetActive(false);
            return;
        }

        if (!_sessionWorldGoalLockValid && !_worldLayoutValid)
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
        if (_sessionWorldGoalLockValid)
            pos = _sessionWorldGoalLockBaseWorld;
        else if (_worldPathFloorPositions.Count > 0)
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
        if (pathSessionActive && !_worldCalibCaptured)
            CaptureWorldSpatialCalibration();

        if (grid.grid == null)
            grid.GenerateGrid();
        else if (regenerateGridEachRepath && CanRegenerateGridFromLiveMap())
            grid.GenerateGrid();

        if (grid.grid == null)
            return;

        AdvanceJourneyProgress();
        ResolveStartEndGrid(out Vector2Int resolvedStart, out Vector2Int resolvedEnd);
        bool mapVisible = IsMapFaceShownForVisuals();
        SetMapVisualsVisible(mapVisible);
        if (mapVisible)
            PlaceEndpointMarkers();

        List<Node> foundPath = pathfinder.FindPath(resolvedStart, resolvedEnd);

        if ((foundPath == null || foundPath.Count == 0) && logPathDiagnostics)
        {
            Vector2 clampedEnd = ClampChartCoordinateToMapBounds(endCoordinate);
            Vector2Int rawEndCell = ChartCoordinateToGridCell(clampedEnd);
            Debug.LogWarning(
                $"[PathTest] No path: chart start={currentCoordinate} end={endCoordinate} " +
                $"endClamped={clampedEnd} mapX=[{mapCoordMin.x},{mapCoordMax.x}] mapY=[{mapCoordMin.y},{mapCoordMax.y}] " +
                $"grid {resolvedStart} (walkable={grid.IsWalkable(resolvedStart)}) rawEndCell={rawEndCell} (walkable={grid.IsWalkable(rawEndCell)}) → " +
                $"resolvedEnd={resolvedEnd} (walkable={grid.IsWalkable(resolvedEnd)}) " +
                $"activeMap={grid.ActiveMapTransform?.name} coordOffset={coordinateOffsetFeet}",
                this);
        }

        currentPath.Clear();
        if (foundPath != null && foundPath.Count > 0)
        {
            EnsurePathRunsStartToEnd(foundPath, resolvedStart, resolvedEnd);
            if (invertPathVisualizationOrder)
                foundPath.Reverse();

            currentPath.AddRange(foundPath);
        }

        RefreshWorldRouteLayoutSnapshot();
        _journeyProgressIndex = 0;

        if (_snapPathNodes.Count > 0)
            AdvanceJourneyProgress();

        if (mapVisible)
        {
            DrawPathLine();
            RebuildMapArrows();
        }
        RebuildWorldArrows();
        _chartGridAtLastRepath = resolvedStart;
        LogPathDiagnostics(resolvedStart, resolvedEnd, mapVisible);
    }

    private void LogPathDiagnostics(Vector2Int resolvedStart, Vector2Int resolvedEnd, bool mapVisible)
    {
        if (!logPathDiagnostics)
            return;

        float distFt = GetRemainingRouteDistanceFeet();
        float worldM = _worldLayoutValid ? PolylineHorizontalLengthMeters(_worldPathFloorPositions) : 0f;
        float mapSurfaceM = _snapPathMapPoints.Count >= 2 ? PolylineArcLengthMeters(_snapPathMapPoints) : 0f;
        float chartJourneyFt = 0f;
        if (_snapPathNodes.Count >= 2)
        {
            float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
            for (int i = 1; i < _snapPathNodes.Count; i++)
            {
                Vector2 a = GridCellToChartCoordinate(_snapPathNodes[i - 1].x, _snapPathNodes[i - 1].y);
                Vector2 b = GridCellToChartCoordinate(_snapPathNodes[i].x, _snapPathNodes[i].y);
                chartJourneyFt += Vector2.Distance(a, b) * toFeet;
            }
        }

        float worldMotionM = 0f;
        string moveTr = "none";
        if (_sessionChartMovementValid)
        {
            Transform tr = ResolveTrackingTransform();
            if (tr != null)
            {
                Vector3 navPos = GetNavigationWorldPosition();
                worldMotionM = (navPos - _sessionTrackingWorldAtCalib).magnitude;
                moveTr = tr.name;
            }
        }

        Vector2 clampedGoal = ClampChartCoordinateToMapBounds(endCoordinate);
        Debug.Log(
            $"[PathTest] repath chartPos={currentCoordinate} startCoord={startCoordinate} endCoord={endCoordinate} endClamped={clampedGoal} " +
            $"grid {resolvedStart.x},{resolvedStart.y} → {resolvedEnd.x},{resolvedEnd.y} pathNodes={currentPath.Count} " +
            $"distance={distFt:F1}ft chartJourney={chartJourneyFt:F1}ft progressIdx={_journeyProgressIndex}/{Mathf.Max(0, _snapPathNodes.Count - 1)} " +
            $"worldPolyline={worldM * feetPerMeter:F1}ft routeT={_sessionRouteProgressT:F2} fullRoute={_sessionFullRouteValid} " +
            $"worldMotion={worldMotionM:F2}m moveTr={moveTr} chartMove={_sessionChartMovementValid} " +
            $"mapVisible={mapVisible} mapArrows={activeMapArrows.Count} worldArrows={activeWorldArrows.Count} goalLock={_sessionWorldGoalLockValid}",
            this);
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
        ChartCoordinateToFractionalGrid(coordinate, out float gx, out float gy);
        return MapWorldFromFractionalGridForChartMovement(gx, gy);
    }

    /// <summary>Chart → world without snapping to integer grid cells (required for movement integration).</summary>
    private Vector3 CoordinateToWorldContinuous(Vector2 coordinate)
    {
        ChartCoordinateToFractionalGrid(coordinate, out float gx, out float gy);
        return MapWorldFromFractionalGridForChartMovement(gx, gy);
    }

    private Vector2 FractionalGridToChartCoordinate(float gx, float gy)
    {
        float denomX = Mathf.Max(1, grid.GridWidth - 1);
        float denomY = Mathf.Max(1, grid.GridHeight - 1);
        float tx = gx / denomX;
        float ty = gy / denomY;
        if (invertCoordinateX) tx = 1f - tx;
        if (invertCoordinateY) ty = 1f - ty;
        tx = Mathf.Clamp01(tx);
        ty = Mathf.Clamp01(ty);

        float adjustedX = Mathf.Lerp(mapCoordMin.x, mapCoordMax.x, tx);
        float adjustedY = Mathf.Lerp(mapCoordMin.y, mapCoordMax.y, ty);
        Vector2 scaled = new Vector2(adjustedX, adjustedY) - coordinateOffsetFeet;
        Vector2 invScale = new Vector2(
            Mathf.Abs(coordinateScale.x) > 1e-8f ? 1f / coordinateScale.x : 0f,
            Mathf.Abs(coordinateScale.y) > 1e-8f ? 1f / coordinateScale.y : 0f);
        return coordinateScalePivot + Vector2.Scale(scaled - coordinateScalePivot, invScale);
    }

    private void ChartCoordinateToFractionalGrid(Vector2 coordinate, out float gx, out float gy)
    {
        Vector2 scaled = coordinateScalePivot + Vector2.Scale(coordinate - coordinateScalePivot, coordinateScale);
        Vector2 adjusted = scaled + coordinateOffsetFeet;
        float tx = Mathf.InverseLerp(mapCoordMin.x, mapCoordMax.x, adjusted.x);
        float ty = Mathf.InverseLerp(mapCoordMin.y, mapCoordMax.y, adjusted.y);

        if (invertCoordinateX) tx = 1f - tx;
        if (invertCoordinateY) ty = 1f - ty;

        tx = Mathf.Clamp01(tx);
        ty = Mathf.Clamp01(ty);

        gx = tx * (grid.GridWidth - 1);
        gy = ty * (grid.GridHeight - 1);
    }

    private Vector3 MapWorldFromFractionalGridForChartMovement(float gx, float gy)
    {
        if (grid == null)
            return Vector3.zero;
        if (UseSessionFrozenMapSpatial())
        {
            if (_worldCalibIsRect)
                return grid.GridToWorldUsingRectCornersFractional(
                    gx, gy, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            float cw = _worldCalibMapW / Mathf.Max(1, _worldCalibGw);
            float ch = _worldCalibMapH / Mathf.Max(1, _worldCalibGh);
            Vector3 bl = _worldCalibCenter - _worldCalibAxisX * (_worldCalibMapW * 0.5f) - _worldCalibAxisY * (_worldCalibMapH * 0.5f);
            return bl + _worldCalibAxisX * ((gx + 0.5f) * cw) + _worldCalibAxisY * ((gy + 0.5f) * ch);
        }
        if (!CanRegenerateGridFromLiveMap() && _worldCalibCaptured)
        {
            if (_worldCalibIsRect)
                return grid.GridToWorldUsingRectCornersFractional(
                    gx, gy, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
            float cw = _worldCalibMapW / Mathf.Max(1, _worldCalibGw);
            float ch = _worldCalibMapH / Mathf.Max(1, _worldCalibGh);
            Vector3 bl = _worldCalibCenter - _worldCalibAxisX * (_worldCalibMapW * 0.5f) - _worldCalibAxisY * (_worldCalibMapH * 0.5f);
            return bl + _worldCalibAxisX * ((gx + 0.5f) * cw) + _worldCalibAxisY * ((gy + 0.5f) * ch);
        }
        return grid.GridToWorldFractional(gx, gy);
    }

    private Vector2Int ChartCoordinateToGridCell(Vector2 coordinate)
    {
        Vector2 scaled = coordinateScalePivot + Vector2.Scale(coordinate - coordinateScalePivot, coordinateScale);
        Vector2 adjusted = scaled + coordinateOffsetFeet;
        float tx = Mathf.InverseLerp(mapCoordMin.x, mapCoordMax.x, adjusted.x);
        float ty = Mathf.InverseLerp(mapCoordMin.y, mapCoordMax.y, adjusted.y);
        if (invertCoordinateX) tx = 1f - tx;
        if (invertCoordinateY) ty = 1f - ty;
        tx = Mathf.Clamp01(tx);
        ty = Mathf.Clamp01(ty);
        int gx = Mathf.RoundToInt(tx * (grid.GridWidth - 1));
        int gy = Mathf.RoundToInt(ty * (grid.GridHeight - 1));
        return new Vector2Int(
            Mathf.Clamp(gx, 0, grid.GridWidth - 1),
            Mathf.Clamp(gy, 0, grid.GridHeight - 1));
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
        return MapFaceOutForSession() * meters * s;
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
        Vector3 startBase = MapCellWorldSnapped(_lineResolvedStart.x, _lineResolvedStart.y);
        Vector3 endBase = MapCellWorldSnapped(_lineResolvedEnd.x, _lineResolvedEnd.y);
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
            Vector3 cell = MapCellWorldSnapped(n.x, n.y);
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

    private static float PolylineHorizontalLengthMeters(IReadOnlyList<Vector3> points)
    {
        if (points == null || points.Count < 2)
            return 0f;
        float sum = 0f;
        for (int i = 1; i < points.Count; i++)
        {
            Vector3 d = points[i] - points[i - 1];
            d.y = 0f;
            sum += d.magnitude;
        }
        return sum;
    }

    /// <summary>Full 3D length along the polyline (required for arrows on a tilted map — horizontal flattening reads ~0).</summary>
    private static float PolylineArcLengthMeters(IReadOnlyList<Vector3> points)
    {
        if (points == null || points.Count < 2)
            return 0f;
        float sum = 0f;
        for (int i = 1; i < points.Count; i++)
            sum += Vector3.Distance(points[i], points[i - 1]);
        return sum;
    }

    private static bool SamplePolylineAtDistance(
        IReadOnlyList<Vector3> points,
        float distanceMeters,
        out Vector3 position,
        out Vector3 segmentTangent)
    {
        position = default;
        segmentTangent = Vector3.forward;
        if (points == null || points.Count == 0)
            return false;
        if (points.Count == 1)
        {
            position = points[0];
            return true;
        }

        float remaining = Mathf.Max(0f, distanceMeters);
        for (int i = 1; i < points.Count; i++)
        {
            Vector3 a = points[i - 1];
            Vector3 b = points[i];
            Vector3 seg = b - a;
            float segLen = seg.magnitude;
            if (segLen < 1e-8f)
                continue;
            if (remaining <= segLen)
            {
                float t = remaining / segLen;
                position = Vector3.Lerp(a, b, t);
                segmentTangent = seg;
                return true;
            }
            remaining -= segLen;
        }

        position = points[points.Count - 1];
        Vector3 tail = points[points.Count - 1] - points[points.Count - 2];
        segmentTangent = tail.sqrMagnitude > 1e-10f ? tail : Vector3.forward;
        return true;
    }

    private static bool SamplePolylineHorizontalAtDistance(
        IReadOnlyList<Vector3> points,
        float distanceMeters,
        out Vector3 position,
        out Vector3 segmentTangent)
    {
        position = default;
        segmentTangent = Vector3.forward;
        if (points == null || points.Count == 0)
            return false;
        if (points.Count == 1)
        {
            position = points[0];
            return true;
        }

        float remaining = Mathf.Max(0f, distanceMeters);
        for (int i = 1; i < points.Count; i++)
        {
            Vector3 a = points[i - 1];
            Vector3 b = points[i];
            Vector3 seg = b - a;
            seg.y = 0f;
            float segLen = seg.magnitude;
            if (segLen < 1e-8f)
                continue;
            if (remaining <= segLen)
            {
                float t = remaining / segLen;
                position = Vector3.Lerp(a, b, t);
                segmentTangent = seg;
                return true;
            }
            remaining -= segLen;
        }

        position = points[points.Count - 1];
        Vector3 tail = points[points.Count - 1] - points[points.Count - 2];
        tail.y = 0f;
        segmentTangent = tail.sqrMagnitude > 1e-10f ? tail : Vector3.forward;
        return true;
    }

    private float PathArrowSpacingMeters() =>
        Mathf.Max(0.5f, pathArrowSpacingFeet) / Mathf.Max(0.01f, feetPerMeter);

    private void RebuildMapArrows()
    {
        DestroyArrowList(activeMapArrows);

        if (mapArrowPrefab == null || currentPath.Count < 2 || grid == null || !IsMapFaceShownForVisuals())
            return;

        bool useJourneySnap = _snapPathNodes.Count == currentPath.Count && _snapPathNodes.Count >= 2;
        Transform parent = EnsureMapArrowsRuntimeRoot();
        float arrowAmount = mapArrowHeightOffset >= 0f ? mapArrowHeightOffset : lineHeightOffset;
        Vector3 arrowLift = MapFaceLift(arrowAmount);
        Vector3 faceNormal = MapFaceOutForSession();

        if (useJourneySnap)
        {
            if (SpawnMapArrowsAlongChartJourney(_snapPathNodes, parent, arrowLift, faceNormal) > 0)
            {
                _lastMapVisualChartCoord = currentCoordinate;
                return;
            }
        }

        var nodes = new List<Node>(currentPath.Count);
        for (int i = 0; i < currentPath.Count; i++)
            nodes.Add(GetCurrentPathNodeJourneyOrder(i));
        if (SpawnMapArrowsAlongChartJourney(nodes, parent, arrowLift, faceNormal) > 0)
            return;

        int step = Mathf.Max(1, mapArrowEveryNNodes);
        for (int i = 0; i < nodes.Count - 1; i += step)
        {
            int next = Mathf.Min(i + step, nodes.Count - 1);
            Vector3 a = MapCellWorldSnapped(nodes[i].x, nodes[i].y);
            Vector3 b = MapCellWorldSnapped(nodes[next].x, nodes[next].y);
            Vector3 direction = b - a;
            if (direction.sqrMagnitude <= Mathf.Epsilon)
                continue;
            SpawnOneMapArrow(a + arrowLift, direction.normalized, faceNormal, parent, $"map node {i}");
        }
    }

    /// <summary>Space arrows by chart feet along the A* node list; world positions come from the live map face per cell.</summary>
    private int SpawnMapArrowsAlongChartJourney(IReadOnlyList<Node> nodes, Transform parent, Vector3 arrowLift, Vector3 faceNormal)
    {
        if (nodes == null || nodes.Count < 2)
            return 0;

        float spacingFt = Mathf.Max(0.5f, pathArrowSpacingFeet);
        float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
        int count = 0;
        float chartSinceSpawn = 0f;
        int lastSpawnIdx = 0;

        int firstAim = Mathf.Min(1, nodes.Count - 1);
        Vector3 startPos = MapCellWorldSnapped(nodes[0].x, nodes[0].y);
        Vector3 firstAimPos = MapCellWorldSnapped(nodes[firstAim].x, nodes[firstAim].y);
        Vector3 startDir = firstAimPos - startPos;
        if (startDir.sqrMagnitude > 1e-12f)
        {
            SpawnOneMapArrow(startPos + arrowLift, startDir.normalized, faceNormal, parent, "chart start");
            count++;
        }

        for (int i = 1; i < nodes.Count; i++)
        {
            Vector2 prevChart = GridCellToChartCoordinate(nodes[i - 1].x, nodes[i - 1].y);
            Vector2 curChart = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            chartSinceSpawn += Vector2.Distance(prevChart, curChart) * toFeet;

            bool isLast = i == nodes.Count - 1;
            if (chartSinceSpawn < spacingFt && !isLast)
                continue;

            int aimIdx = isLast ? i : Mathf.Min(i + 1, nodes.Count - 1);
            Vector3 spawnPos = MapCellWorldSnapped(nodes[lastSpawnIdx].x, nodes[lastSpawnIdx].y);
            Vector3 aimPos = MapCellWorldSnapped(nodes[aimIdx].x, nodes[aimIdx].y);
            Vector3 direction = aimPos - spawnPos;
            if (direction.sqrMagnitude <= 1e-12f && aimIdx + 1 < nodes.Count)
                aimPos = MapCellWorldSnapped(nodes[aimIdx + 1].x, nodes[aimIdx + 1].y);
            direction = aimPos - spawnPos;
            if (direction.sqrMagnitude <= Mathf.Epsilon)
                continue;

            SpawnOneMapArrow(spawnPos + arrowLift, direction.normalized, faceNormal, parent, $"chart seg {lastSpawnIdx}");
            count++;
            chartSinceSpawn = 0f;
            lastSpawnIdx = i;
        }

        return count;
    }

    private int SpawnMapArrowsAlongPolyline(List<Vector3> polyline, Transform parent, Vector3 arrowLift, Vector3 faceNormal)
    {
        float spacingM = PathArrowSpacingMeters();
        float totalM = PolylineArcLengthMeters(polyline);
        if (totalM < 1e-4f)
            return 0;

        int count = 0;
        for (float dist = 0f; dist < totalM - 0.02f; dist += spacingM)
        {
            if (!SamplePolylineAtDistance(polyline, dist, out Vector3 worldPos, out _))
                continue;
            float aimDist = Mathf.Min(dist + spacingM, totalM);
            if (!SamplePolylineAtDistance(polyline, aimDist, out Vector3 aimPos, out Vector3 segTan))
                continue;

            Vector3 direction = aimPos - worldPos;
            if (direction.sqrMagnitude <= 1e-12f && segTan.sqrMagnitude > 1e-12f)
                direction = segTan;
            if (direction.sqrMagnitude <= Mathf.Epsilon)
                continue;
            SpawnOneMapArrow(worldPos + arrowLift, direction.normalized, faceNormal, parent, $"map d={dist:F1}m");
            count++;
        }
        return count;
    }

    private void SpawnOneMapArrow(Vector3 spawnPos, Vector3 direction, Vector3 faceNormal, Transform parent, string suffix)
    {
        Quaternion rotation = Quaternion.AngleAxis(mapArrowRollAroundNormalDegrees, faceNormal)
            * Quaternion.LookRotation(direction, faceNormal);
        GameObject instance = Instantiate(mapArrowPrefab);
        instance.name = $"{mapArrowPrefab.name} ({suffix})";
        instance.SetActive(true);
        Transform t = instance.transform;
        Vector3 desiredWorldScale = t.lossyScale;
        t.SetPositionAndRotation(spawnPos, rotation);
        t.SetParent(parent, true);
        PreserveChildLossyScale(t, desiredWorldScale);
        activeMapArrows.Add(instance);
    }

    /// <summary>Call when starting a path session or pressing Orient North. Locks world anchor + map basis + north; timer repath does not move the trail with the user.</summary>
    private void CaptureWorldSpatialCalibration(bool resumeExistingProgress = false)
    {
        if (!resumeExistingProgress)
        {
            _worldCalibCaptured = false;
            _worldLayoutValid = false;
        }

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
        if (!resumeExistingProgress)
            RefreshSessionChartMovementOrigin();
    }

    private Vector3 GetWorldPathFloorAnchor() => GetWorldPathFloorAnchor(null);

    /// <summary>Per repath: snapshot the current map polyline so world arrows follow the same route geometry.</summary>
    private void RefreshWorldRouteLayoutSnapshot()
    {
        if (pathSessionActive && !_worldCalibCaptured)
            CaptureWorldSpatialCalibration();

        _worldLayoutValid = false;
        _snapPathMapPoints.Clear();
        _snapPathNodes.Clear();
        if (grid == null || currentPath.Count < 1 || !_worldCalibCaptured)
            return;

        for (int i = 0; i < currentPath.Count; i++)
        {
            Node n = currentPath[i];
            _snapPathNodes.Add(n);
            Vector3 mapPoint = MapCellWorldSnapped(n.x, n.y);
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
                    Vector3 startSnap = MapCellWorldSnapped(_lineResolvedStart.x, _lineResolvedStart.y);
                    Vector3 endSnap = MapCellWorldSnapped(_lineResolvedEnd.x, _lineResolvedEnd.y);
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

        if (_sessionFullRouteValid)
        {
            ApplySessionRouteProgressTrimmedVisuals();
            if (_sessionWorldGoalLockValid && !_sessionRouteStartFeetValid)
            {
                _sessionRouteStartFeetWorld = GetNavigationWorldPosition();
                _sessionRouteStartFeetValid = true;
            }
            return;
        }

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
            //
            // IMPORTANT: worldPathYawOffsetDegrees and flipWorldFloorDefaultDirection are *default-direction overrides*
            // (used when mapN happens to point the wrong way at scene start). Once the user has explicitly locked a
            // direction with Orient North, that lock is the source of truth and these manual overrides MUST be ignored,
            // otherwise they stack on top of the lock and flip the path 180° away from where the user calibrated.
            float extraYaw;
            if (worldFloorUsesGridLayout)
            {
                extraYaw = northOrientationLocked ? 0f : worldPathYawOffsetDegrees;
            }
            else
            {
                extraYaw = worldPathYawOffsetDegrees + _worldCalibNorthYawDeg;
            }
            if (flipWorldFloorDefaultDirection && !northOrientationLocked)
                extraYaw += 180f;
            Quaternion qNorth = Quaternion.Euler(0f, extraYaw, 0f);
            w1 = w0 + qNorth * rawFlat[rawFlat.Count - 1];
        }

        if (!_sessionWorldGoalLockValid && rawFlat.Count > 0)
        {
            float lockYaw = worldFloorUsesGridLayout
                ? (northOrientationLocked ? 0f : worldPathYawOffsetDegrees)
                : worldPathYawOffsetDegrees + _worldCalibNorthYawDeg;
            if (flipWorldFloorDefaultDirection && !northOrientationLocked)
                lockYaw += 180f;
            Vector3 lockOffset = Quaternion.Euler(0f, lockYaw, 0f) * rawFlat[rawFlat.Count - 1];

            float chartPathFt = 0f;
            float toFeet = Mathf.Max(0f, chartDistanceUnitsToFeet);
            for (int i = 1; i < _snapPathNodes.Count; i++)
            {
                Vector2 a = GridCellToChartCoordinate(_snapPathNodes[i - 1].x, _snapPathNodes[i - 1].y);
                Vector2 b = GridCellToChartCoordinate(_snapPathNodes[i].x, _snapPathNodes[i].y);
                chartPathFt += Vector2.Distance(a, b) * toFeet;
            }

            float layoutMag = lockOffset.magnitude;
            if (layoutMag > 1e-6f && chartPathFt > 0.1f)
                lockOffset = lockOffset.normalized * (chartPathFt / Mathf.Max(0.01f, feetPerMeter));

            _sessionWorldGoalLockBaseWorld = GetWorldPathFloorAnchor() + lockOffset + Vector3.up * worldArrowHeightAbovePlayer;
            _sessionWorldGoalLockEndCell = _lineResolvedEnd;
            _sessionWorldGoalLockValid = true;
        }

        // With a session-frozen world goal, w0 tracks the rig while the red marker stays fixed. If we keep
        // w1 = w0 + (const offset), then w1 - w0 is translation-invariant and the entire floor polyline slides with the player.
        // Retie the stretch target to the locked goal on the floor so arrows update from moving feet toward a fixed endpoint.
        if (_sessionWorldGoalLockValid && worldRouteGoalWorld == null
            && _lineResolvedEnd == _sessionWorldGoalLockEndCell)
        {
            Vector3 lockedFloor = _sessionWorldGoalLockBaseWorld - Vector3.up * worldArrowHeightAbovePlayer;
            w1 = lockedFloor;
            w1.y = w0.y;
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

        if (_sessionWorldGoalLockValid && !_sessionRouteStartFeetValid)
        {
            _sessionRouteStartFeetWorld = GetNavigationWorldPosition();
            _sessionRouteStartFeetValid = true;
        }

        if (!_sessionFullRouteValid)
            TryCaptureSessionFullRoute(_snapPathMapPoints);
    }

    private Vector3 GridToWorldSnapshot(int gx, int gy)
    {
        return GridToWorldSnapshotFractional(gx, gy);
    }

    private Vector3 GridToWorldSnapshotFractional(float gx, float gy)
    {
        if (!_worldCalibCaptured)
            return _worldCalibAnchor;
        if (_worldCalibIsRect)
            return grid.GridToWorldUsingRectCornersFractional(
                gx, gy, _worldCalibCorner0, _worldCalibCorner1, _worldCalibCorner3);
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
        Vector3 p = MapCellWorldSnapped(gx, gy);
        return WorldArrowFloorFromSnapshotMapPoint(p);
    }

    /// <summary>Nearest point on the floor route polyline to a map-face sample (counts may differ after route trim).</summary>
    private Vector3 WorldArrowFloorFromSnapshotMapPoint(Vector3 mapPointWorld)
    {
        if (!_worldLayoutValid || !_worldCalibCaptured || _worldPathFloorPositions.Count < 1)
            return GetWorldPathFloorAnchor() + Vector3.up * worldArrowHeightAbovePlayer;

        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < _worldPathFloorPositions.Count; i++)
        {
            float d = (_worldPathFloorPositions[i] - mapPointWorld).sqrMagnitude;
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

        if (!_worldLayoutValid || _worldPathFloorPositions.Count < 2)
        {
            if (pathSessionActive && currentPath.Count >= 1 && _worldCalibCaptured)
                RefreshWorldRouteLayoutSnapshot();
        }

        if (worldArrowPrefab == null || grid == null || grid.grid == null
            || !_worldCalibCaptured || !_worldLayoutValid || _worldPathFloorPositions.Count < 2)
            return;

        float spaceScale = Mathf.Max(0.001f, worldPathUniformSpaceScale);
        Vector3 visualScaleMul = worldArrowMeshScalesWithPath
            ? Vector3.Scale(worldArrowScaleMultiplier, new Vector3(spaceScale, spaceScale, spaceScale))
            : worldArrowScaleMultiplier;

        float spacingM = PathArrowSpacingMeters();
        float totalM = PolylineHorizontalLengthMeters(_worldPathFloorPositions);
        if (totalM < 1e-4f)
            return;

        Quaternion playerMeshYawOnly = Quaternion.Euler(0f, worldArrowLookYawOffsetDegrees, 0f);
        Transform tracking = ResolveTrackingTransform();

        if (snapPlayerYawToWorldPath && _worldPathFloorPositions.Count >= 2 && tracking != null)
        {
            float aimDist = Mathf.Min(spacingM, totalM);
            if (SamplePolylineHorizontalAtDistance(_worldPathFloorPositions, 0f, out Vector3 p0, out _)
                && SamplePolylineHorizontalAtDistance(_worldPathFloorPositions, aimDist, out Vector3 p1, out _))
            {
                p0 += Vector3.up * worldArrowHeightAbovePlayer;
                p1 += Vector3.up * worldArrowHeightAbovePlayer;
                Vector3 flat = Vector3.ProjectOnPlane(p1 - p0, Vector3.up);
                if (flat.sqrMagnitude > 1e-8f)
                {
                    Quaternion facePath = Quaternion.LookRotation(flat.normalized, Vector3.up) * playerMeshYawOnly;
                    Vector3 e = tracking.eulerAngles;
                    e.y = facePath.eulerAngles.y;
                    tracking.eulerAngles = e;
                }
            }
        }

        Vector3 lift = Vector3.up * worldArrowHeightAbovePlayer;
        for (float dist = 0f; dist < totalM - 0.02f; dist += spacingM)
        {
            if (!SamplePolylineHorizontalAtDistance(_worldPathFloorPositions, dist, out Vector3 floorPos, out _))
                continue;
            float aimDist = Mathf.Min(dist + spacingM, totalM);
            if (!SamplePolylineHorizontalAtDistance(_worldPathFloorPositions, aimDist, out Vector3 aimPos, out Vector3 segTan))
                continue;

            Vector3 worldPos = floorPos + lift;
            Vector3 tangent = Vector3.ProjectOnPlane(aimPos - floorPos, Vector3.up);
            if (tangent.sqrMagnitude <= 1e-12f)
                tangent = Vector3.ProjectOnPlane(segTan, Vector3.up);
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

    /// <summary>World-space goal for the floor marker and off-screen cue: session-frozen polyline tail while the goal cell is unchanged; otherwise same as live tail.</summary>
    private bool TryGetWorldPathDestinationWorld(out Vector3 worldPos)
    {
        worldPos = default;
        if (!pathSessionActive || grid == null || grid.grid == null)
            return false;
        if (_sessionWorldGoalLockValid)
        {
            worldPos = _sessionWorldGoalLockBaseWorld + Vector3.up * endPointMarkerHeightAboveRoute;
            return true;
        }

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

    private bool EnsureOffScreenTurnCuePair(Transform parent, GameObject prefab)
    {
        if (parent == null || prefab == null)
            return false;

        bool parentChanged = _offScreenCueLeft != null && _offScreenCueLeft.transform.parent != parent
            || _offScreenCueRight != null && _offScreenCueRight.transform.parent != parent;
        if (parentChanged)
            DestroyOffScreenTurnCueInstances();

        if (_offScreenCueLeft == null)
        {
            _offScreenCueLeft = Instantiate(prefab, parent);
            _offScreenCueLeft.name = "OffScreenTurnCue_Left";
        }
        else if (_offScreenCueLeft.transform.parent != parent)
            _offScreenCueLeft.transform.SetParent(parent, false);

        if (_offScreenCueRight == null)
        {
            _offScreenCueRight = Instantiate(prefab, parent);
            _offScreenCueRight.name = "OffScreenTurnCue_Right";
        }
        else if (_offScreenCueRight.transform.parent != parent)
            _offScreenCueRight.transform.SetParent(parent, false);

        bool hudRect = IsHudRectParent(parent);
        if (hudRect)
        {
            ConfigureHudRectCueInstance(_offScreenCueLeft);
            ConfigureHudRectCueInstance(_offScreenCueRight);
        }
        else
        {
            _offScreenCueLeft.transform.localScale = offScreenTurnCueLocalScale;
            _offScreenCueRight.transform.localScale = offScreenTurnCueLocalScale;
        }

        return true;
    }

    private Transform ResolveOffScreenCueParentTransform(Camera cam)
    {
        if (offScreenTurnCueHudAnchor != null && offScreenTurnCueHudAnchor.gameObject.activeInHierarchy)
            return offScreenTurnCueHudAnchor;
        if (parentOffScreenTurnCueToDistanceReadout)
        {
            // Prefer any readout that is still in an active hierarchy (Navigation sidebar may be disabled while path runs).
            foreach (TextMeshProUGUI tmp in EnumerateDistanceReadouts())
            {
                if (tmp != null && tmp.gameObject.activeInHierarchy)
                    return tmp.rectTransform;
            }
        }
        return cam != null ? cam.transform : null;
    }

    private bool IsHudRectParent(Transform parent)
    {
        return parent != null && parent is RectTransform;
    }

    private void ConfigureHudRectCueInstance(GameObject cue)
    {
        if (cue == null)
            return;
        var rt = cue.GetComponent<RectTransform>();
        if (rt == null)
            rt = cue.AddComponent<RectTransform>();
        rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = Vector2.zero;
        rt.localScale = Vector3.one * Mathf.Max(0.01f, offScreenTurnCueHudMeshUniformScale);
        if (offScreenTurnCueHudMeshSortingOrder != 0)
        {
            var renderers = cue.GetComponentsInChildren<MeshRenderer>(true);
            for (int i = 0; i < renderers.Length; i++)
                renderers[i].sortingOrder = offScreenTurnCueHudMeshSortingOrder;
        }
    }

    private Quaternion ComputeOffScreenCueAimWorld(Camera cam, bool turnRight)
    {
        Vector3 dirWorld = turnRight ? cam.transform.right : -cam.transform.right;
        return Quaternion.AngleAxis(worldArrowCompassYawDegrees, Vector3.up)
            * WorldArrowBaseRotation(dirWorld, worldArrowMeshForwardAxis)
            * Quaternion.Euler(worldArrowMeshPitchDegrees, worldArrowLookYawOffsetDegrees, 0f)
            * Quaternion.Euler(offScreenTurnCueRotationExtraEuler);
    }

    private void ApplyOffScreenCueLayout(GameObject show, Camera cam, Transform parent, bool hudRectMode, bool turnRight)
    {
        if (show == null || cam == null || parent == null)
            return;

        Quaternion aimWorld = ComputeOffScreenCueAimWorld(cam, turnRight);
        if (hudRectMode && parent is RectTransform)
        {
            var rt = show.GetComponent<RectTransform>();
            if (rt == null)
            {
                ConfigureHudRectCueInstance(show);
                rt = show.GetComponent<RectTransform>();
            }
            float ax = Mathf.Abs(offScreenTurnCueHudAnchoredOffsetX);
            rt.anchoredPosition = new Vector2(turnRight ? ax : -ax, offScreenTurnCueHudAnchoredOffsetY);
            rt.localRotation = Quaternion.Inverse(parent.rotation) * aimWorld;
            show.transform.SetAsLastSibling();
        }
        else
        {
            float x = Mathf.Abs(offScreenTurnCueLocalX);
            show.transform.localPosition = turnRight
                ? new Vector3(x, offScreenTurnCueLocalY, offScreenTurnCueLocalDepth)
                : new Vector3(-x, offScreenTurnCueLocalY, offScreenTurnCueLocalDepth);
            show.transform.localRotation = Quaternion.Inverse(cam.transform.rotation) * aimWorld;
        }
    }

    private void UpdateOffScreenTurnCues()
    {
        GameObject prefab = offScreenTurnCuePrefab != null ? offScreenTurnCuePrefab : worldArrowPrefab;
        if (prefab == null)
        {
            HideOffScreenTurnCues();
            return;
        }

        if (!TryGetWorldPathDestinationWorld(out Vector3 destWorld))
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

        Transform cueParent = ResolveOffScreenCueParentTransform(cam);
        if (cueParent == null)
        {
            HideOffScreenTurnCues();
            return;
        }

        bool hudRectMode = IsHudRectParent(cueParent);
        if (!EnsureOffScreenTurnCuePair(cueParent, prefab))
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

        ApplyOffScreenCueLayout(show, cam, cueParent, hudRectMode, turnRightIsShorter);
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
        bool noPath = currentPath.Count < 2;
        string line;
        if (noPath)
            line = noPathAvailableText;
        else if (remainingFeet <= Mathf.Max(0f, destinationReachedFeet))
            line = destinationReachedText;
        else
            line = $"Distance: {remainingFeet:F1} ft";
        foreach (TextMeshProUGUI tmp in EnumerateDistanceReadouts())
        {
            tmp.enabled = true;
            tmp.text = line;
            EnsureDistanceLabelRenderable(tmp);
            if (!tmp.gameObject.activeInHierarchy)
                tmp.gameObject.SetActive(true);
            tmp.ForceMeshUpdate(true);
        }

        if (pathSessionActive && HasAnyDistanceReadout())
            Canvas.ForceUpdateCanvases();
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

        if (distanceSumUsesRawPathListOrder)
        {
            float totalChart = 0f;
            Vector2 previous = currentCoordinate;
            for (int i = 0; i < currentPath.Count; i++)
            {
                Node stepNode = currentPath[i];
                Vector2 current = GridCellToChartCoordinate(stepNode.x, stepNode.y);
                totalChart += Vector2.Distance(previous, current);
                previous = current;
            }
            return totalChart * toFeet;
        }

        // Remaining distance along journey from current chart position toward goal (grows again when backing away).
        IReadOnlyList<Node> journeyNodes = GetJourneyNodesForDistance();
        if (journeyNodes != null && journeyNodes.Count >= 2)
            return RemainingChartDistanceFromProbeToGoalFeet(journeyNodes);

        return Vector2.Distance(currentCoordinate, endCoordinate) * toFeet;
    }

    private float RemainingChartDistanceFromProbeToGoalFeet(IReadOnlyList<Node> nodes)
    {
        float scale = Mathf.Max(0f, chartDistanceUnitsToFeet);
        int ix = FindClosestJourneyIndexOnList(nodes);
        Vector2 cIx = GridCellToChartCoordinate(nodes[ix].x, nodes[ix].y);
        float sum = Vector2.Distance(currentCoordinate, cIx);
        for (int i = ix; i < nodes.Count - 1; i++)
        {
            Vector2 a = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            Vector2 b = GridCellToChartCoordinate(nodes[i + 1].x, nodes[i + 1].y);
            sum += Vector2.Distance(a, b);
        }

        return sum * scale;
    }

    private IReadOnlyList<Node> GetJourneyNodesForDistance()
    {
        if (_snapPathNodes.Count == currentPath.Count && _snapPathNodes.Count > 0)
            return _snapPathNodes;
        if (currentPath.Count == 0)
            return null;
        var ordered = new List<Node>(currentPath.Count);
        for (int i = 0; i < currentPath.Count; i++)
            ordered.Add(GetCurrentPathNodeJourneyOrder(i));
        return ordered;
    }

    private int FindClosestJourneyIndexOnList(IReadOnlyList<Node> nodes)
    {
        if (nodes == null || nodes.Count == 0)
            return 0;
        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector2 c = GridCellToChartCoordinate(nodes[i].x, nodes[i].y);
            float d = (c - currentCoordinate).sqrMagnitude;
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        return best;
    }
}
