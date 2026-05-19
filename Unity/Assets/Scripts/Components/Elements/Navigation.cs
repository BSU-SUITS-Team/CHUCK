using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using MixedReality.Toolkit.UX;

public class Navigation : MonoBehaviour
{
    /// <summary>Most recently enabled Navigation panel — used to refresh GridManager when starting a path from an arbitrary pose.</summary>
    private static Navigation s_lastActiveInstance;

    [SerializeField] RectTransform image;
    [SerializeField] RectTransform map;
    [SerializeField] ARSIS.UI.Button toggleCapture;
    [SerializeField, Tooltip("Optional. If unset, uses PathTest.Instance for Find Path / Stop Path.")]
    private PathTest pathTest;
    [SerializeField, Tooltip("Optional north marker on this prefab (e.g. empty under Map). Copied to PathTest.mapNorthReference whenever this panel syncs. Leave empty to use PathTest’s own assignment or default map-up north.")]
    private Transform mapNorthReference;
    [SerializeField, Tooltip("Optional distance line. Leave empty to auto-create PathDistanceReadout under the Controls column when ensurePathDistanceInSidebar is on.")]
    private TextMeshProUGUI pathDistanceReadout;
    [SerializeField, Tooltip("Adds a row under Find Path on the Navigation panel and binds it to PathTest.")]
    private bool ensurePathDistanceInSidebar = true;

    [Header("Route end presets (chart feet)")]
    [SerializeField, Tooltip("Route goal when the cycle button shows A.")]
    private Vector2 aCoordinate = new Vector2(-5635f, -9960f);
    [SerializeField, Tooltip("Route goal B.")]
    private Vector2 bCoordinate = new Vector2(-5615f, -9995f);
    [SerializeField, Tooltip("Route goal HAB.")]
    private Vector2 habCoordinate = new Vector2(-5670f, -10060f);
    [SerializeField, Tooltip("Active preset (A → B → HAB). Pushed to PathTest on enable and when edited.")]
    private RouteEndSelection routeEndSelection = RouteEndSelection.A;

    private bool isCapture = false;
    private bool findPathButtonSynced;

    private const float PrimaryNavButtonMinHeight = 72f;
    private const float PrimaryNavButtonPreferredHeight = 96f;
    private const float NavButtonRowMinHeight = 72f;
    private const float NavButtonRowPreferredHeight = 96f;

    private PathTest ResolvePathTest() => pathTest != null ? pathTest : PathTest.Instance;

    /// <summary>Call when PathTest starts a session so the grid uses this menu’s map plane (same as at Start / after floating placement).</summary>
    public static void RefreshPathfindingMapBindingStatic()
    {
        if (s_lastActiveInstance != null)
        {
            s_lastActiveInstance.SyncPathfindingToActiveMapPlane();
            return;
        }

        Navigation[] instances = Object.FindObjectsByType<Navigation>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < instances.Length; i++)
        {
            Navigation nav = instances[i];
            if (nav != null && nav.isActiveAndEnabled)
            {
                nav.SyncPathfindingToActiveMapPlane();
                return;
            }
        }
    }

    private void OnEnable()
    {
        s_lastActiveInstance = this;
        SyncRoutePresetsToPathTest();
        SyncPathfindingToActiveMapPlane();
    }

    private void OnValidate()
    {
        SyncRoutePresetsToPathTest();
    }

    private void SyncRoutePresetsToPathTest()
    {
        PathTest pt = ResolvePathTest();
        if (pt == null)
            return;
        pt.ApplyRoutePresetsFromNavigation(aCoordinate, bCoordinate, habCoordinate, routeEndSelection);
    }

    private void OnDisable()
    {
        // Only the panel that currently drives PathTest's map binding should hide map-face visuals when it closes.
        bool wasActiveBinder = s_lastActiveInstance == this;
        if (wasActiveBinder)
        {
            PathTest pt = ResolvePathTest();
            if (pt != null)
                pt.NotifyNavigationPanelHidden();
            s_lastActiveInstance = null;
        }
    }

    private void SetCaptureButton(bool isPathCapture)
    {
        if (isCapture == isPathCapture) return;
        string icon = isPathCapture ? "Icon 135" : "Icon 128";
        string label = isPathCapture ? "Stop Path" : "Record Path";
        toggleCapture.SetIcon(true, icon, label);
        isCapture = isPathCapture;
    }

    private void SyncFindPathButton()
    {
        PathTest pt = ResolvePathTest();
        if (pt == null)
            return;
        bool session = pt.IsPathSessionActive();
        if (findPathButtonSynced == session)
            return;
        findPathButtonSynced = session;
        string icon = session ? "Icon 135" : "Icon 128";
        string label = session ? "Stop Path" : "Find Path";
        toggleCapture.SetIcon(true, icon, label);
    }

    /// <summary>Assume you are facing chart north in world space; stores map→world north yaw on PathTest (does not toggle pathfinding).</summary>
    public void OrientNorthFromMenu()
    {
        PathTest pt = ResolvePathTest();
        pt?.CalibrateNorthFromUserFacing();
    }

    /// <summary>Cycle route preset A → B → HAB; updates PathTest end coordinate and repaths when active.</summary>
    public void CycleRouteEndFromMenu()
    {
        PathTest pt = ResolvePathTest();
        if (pt == null)
            return;
        SyncRoutePresetsToPathTest();
        pt.CycleRouteEndSelection();
        routeEndSelection = pt.ActiveRouteEndSelection;
        RefreshRouteEndButtonLabel();
    }

    public void TogglePathCapture()
    {
        PathTest pt = ResolvePathTest();
        if (pt != null)
        {
            SyncPathfindingToActiveMapPlane();
            pt.SetPathSessionActive(!pt.IsPathSessionActive());
            findPathButtonSynced = pt.IsPathSessionActive();
            string icon = findPathButtonSynced ? "Icon 135" : "Icon 128";
            string label = findPathButtonSynced ? "Stop Path" : "Find Path";
            toggleCapture.SetIcon(true, icon, label);
            return;
        }

        if (TranslationController.S != null)
            TranslationController.S.togglePathCapture();
    }

    void Awake()
    {
        if (pathDistanceReadout == null && ensurePathDistanceInSidebar)
            TryInjectPathDistanceSidebar();
        TryEnsureNavControlButtons();
        ApplyPrimaryNavigationButtonSizing();
    }

    void Start()
    {
        SyncPathfindingToActiveMapPlane();
        PathTest pt = ResolvePathTest();
        if (pathDistanceReadout != null && pt != null)
            pt.BindDistanceReadout(pathDistanceReadout);
        if (pt != null)
        {
            SyncRoutePresetsToPathTest();
            RefreshRouteEndButtonLabel();
            bool session = pt.IsPathSessionActive();
            findPathButtonSynced = session;
            string icon = session ? "Icon 135" : "Icon 128";
            string label = session ? "Stop Path" : "Find Path";
            toggleCapture.SetIcon(true, icon, label);
        }
        else if (TranslationController.S != null)
            SetCaptureButton(TranslationController.S.IsPathCapture());
    }

    void OnDestroy()
    {
        if (pathDistanceReadout != null && PathTest.Instance != null)
            PathTest.Instance.UnbindDistanceReadout(pathDistanceReadout);
    }

    /// <summary>Called after <see cref="FloatingMenuFromPrefab"/> positions the panel so PathTest uses this instance's map plane (not scene origin).</summary>
    public void NotifyMenuPlacedInFrontOfUser()
    {
        SyncPathfindingToActiveMapPlane();
    }

    /// <summary>Binds <see cref="PathTest"/> to this panel's chart plane (editor scene instance or floating menu).</summary>
    public void SyncPathfindingToActiveMapPlane()
    {
        RectTransform chartSurface = ResolveChartSurfaceForPathfinding();
        if (chartSurface == null)
            return;
        PathTest pt = ResolvePathTest();
        if (pt == null)
            return;
        SyncRoutePresetsToPathTest();
        pt.BindActiveNavigationMap(chartSurface);
        if (mapNorthReference != null)
        {
            pt.mapNorthReference = mapNorthReference;
            pt.RefreshMapNorthReferenceVisibility();
        }
    }

    /// <summary>Map viewport rect — same surface GridManager and map markers used before route presets.</summary>
    private RectTransform ResolveChartSurfaceForPathfinding()
    {
        if (map != null)
            return map;
        return image;
    }

    private void TryInjectPathDistanceSidebar()
    {
        RectTransform controls = null;
        foreach (RectTransform rt in GetComponentsInChildren<RectTransform>(true))
        {
            if (rt.name != "Controls")
                continue;
            Transform existingReadout = rt.Find("PathDistanceReadout");
            if (existingReadout != null)
            {
                pathDistanceReadout = existingReadout.GetComponent<TextMeshProUGUI>();
                TryEnsureNavControlButtonsUnderControls(rt);
                return;
            }

            if (rt.GetComponent<UnityEngine.UI.HorizontalLayoutGroup>() != null)
                controls = rt;
        }

        if (controls == null)
            return;

        var horizontal = controls.GetComponent<UnityEngine.UI.HorizontalLayoutGroup>();
        if (horizontal == null)
            return;

        var toMove = new List<Transform>(controls.childCount);
        for (int i = 0; i < controls.childCount; i++)
            toMove.Add(controls.GetChild(i));

        float spacing = horizontal.spacing;
        RectOffset pad = horizontal.padding;
        TextAnchor alignment = horizontal.childAlignment;
        bool ccw = horizontal.childControlWidth;
        bool cch = horizontal.childControlHeight;
        bool few = horizontal.childForceExpandWidth;
        bool feh = horizontal.childForceExpandHeight;
        Destroy(horizontal);

        UnityEngine.UI.VerticalLayoutGroup vertical = controls.gameObject.AddComponent<UnityEngine.UI.VerticalLayoutGroup>();
        vertical.spacing = spacing;
        vertical.padding = new RectOffset(pad.left, pad.right, pad.top, pad.bottom);
        vertical.childAlignment = TextAnchor.UpperCenter;
        vertical.childControlWidth = true;
        vertical.childControlHeight = true;
        vertical.childForceExpandWidth = true;
        vertical.childForceExpandHeight = false;

        var rowGo = new GameObject("NavButtonRow", typeof(RectTransform));
        var rowRt = rowGo.GetComponent<RectTransform>();
        rowRt.SetParent(controls, false);
        rowRt.anchorMin = new Vector2(0f, 1f);
        rowRt.anchorMax = new Vector2(1f, 1f);
        rowRt.pivot = new Vector2(0.5f, 1f);
        rowRt.sizeDelta = Vector2.zero;
        rowRt.anchoredPosition = Vector2.zero;

        var rowHl = rowGo.AddComponent<UnityEngine.UI.HorizontalLayoutGroup>();
        rowHl.spacing = spacing;
        rowHl.padding = new RectOffset(pad.left, pad.right, pad.top, pad.bottom);
        rowHl.childAlignment = alignment;
        rowHl.childControlWidth = ccw;
        rowHl.childControlHeight = cch;
        rowHl.childForceExpandWidth = few;
        rowHl.childForceExpandHeight = feh;

        var rowLe = rowGo.AddComponent<UnityEngine.UI.LayoutElement>();
        rowLe.flexibleWidth = 1f;
        rowLe.minHeight = NavButtonRowMinHeight;
        rowLe.preferredHeight = NavButtonRowPreferredHeight;

        foreach (Transform child in toMove)
            child.SetParent(rowRt, false);

        var distGo = new GameObject("PathDistanceReadout", typeof(RectTransform));
        var distRt = distGo.GetComponent<RectTransform>();
        distRt.SetParent(controls, false);
        distRt.anchorMin = new Vector2(0f, 1f);
        distRt.anchorMax = new Vector2(1f, 1f);
        distRt.pivot = new Vector2(0.5f, 1f);
        distRt.sizeDelta = Vector2.zero;

        var le = distGo.AddComponent<UnityEngine.UI.LayoutElement>();
        le.preferredHeight = 40f;
        le.minHeight = 28f;
        le.flexibleWidth = 1f;

        var tmp = distGo.AddComponent<TextMeshProUGUI>();
        tmp.text = string.Empty;
        tmp.fontSize = 11f;
        tmp.raycastTarget = false;
        tmp.enableWordWrapping = true;
        tmp.overflowMode = TextOverflowModes.Ellipsis;
        tmp.alignment = TextAlignmentOptions.TopLeft;
        tmp.color = Color.white;
        if (TMP_Settings.defaultFontAsset != null)
        {
            tmp.font = TMP_Settings.defaultFontAsset;
            tmp.fontSharedMaterial = TMP_Settings.defaultFontAsset.material;
        }

        rowRt.SetAsFirstSibling();
        distRt.SetAsLastSibling();

        pathDistanceReadout = tmp;
        TryEnsureNavControlButtonsUnderControls(controls);
    }

    private void TryEnsureNavControlButtons()
    {
        foreach (RectTransform rt in GetComponentsInChildren<RectTransform>(true))
        {
            if (rt.name != "Controls")
                continue;
            if (rt.GetComponent<VerticalLayoutGroup>() == null)
                continue;
            TryEnsureNavControlButtonsUnderControls(rt);
            return;
        }
    }

    private void TryEnsureNavControlButtonsUnderControls(RectTransform controls)
    {
        if (controls == null || toggleCapture == null)
            return;

        Transform legacyRow = controls.Find("OrientNorthRow");
        if (legacyRow != null)
            Destroy(legacyRow.gameObject);

        Transform pathParent = toggleCapture.transform.parent;
        if (pathParent == null)
            return;

        int captureIndex = toggleCapture.transform.GetSiblingIndex();

        Transform routeEnd = pathParent.Find("RouteEndButton");
        if (routeEnd == null)
        {
            GameObject routeClone = Instantiate(toggleCapture.gameObject, pathParent);
            routeClone.name = "RouteEndButton";
            routeEnd = routeClone.transform;
        }

        ConfigureRouteEndButton(routeEnd.gameObject);
        routeEnd.SetSiblingIndex(captureIndex + 1);

        Transform orientNorth = pathParent.Find("OrientNorthButton");
        if (orientNorth == null)
        {
            GameObject orientClone = Instantiate(toggleCapture.gameObject, pathParent);
            orientClone.name = "OrientNorthButton";
            orientNorth = orientClone.transform;
        }

        ConfigureOrientNorthButton(orientNorth.gameObject);
        orientNorth.SetSiblingIndex(captureIndex + 2);
    }

    private void RefreshRouteEndButtonLabel()
    {
        if (toggleCapture == null)
            return;

        Transform pathParent = toggleCapture.transform.parent;
        if (pathParent == null)
            return;

        Transform routeEnd = pathParent.Find("RouteEndButton");
        if (routeEnd == null)
            return;

        ARSIS.UI.Button arsisBtn = routeEnd.GetComponent<ARSIS.UI.Button>();
        PathTest pt = ResolvePathTest();
        if (arsisBtn == null || pt == null)
            return;

        arsisBtn.SetIcon(false, string.Empty, string.Empty);
        arsisBtn.SetText(true, pt.GetRouteEndSelectionLabel());
    }

    private void ConfigureRouteEndButton(GameObject buttonObject)
    {
        if (buttonObject == null)
            return;

        ARSIS.UI.Button arsisBtn = buttonObject.GetComponent<ARSIS.UI.Button>();
        PathTest pt = ResolvePathTest();
        if (arsisBtn != null)
        {
            arsisBtn.SetIcon(false, string.Empty, string.Empty);
            arsisBtn.SetText(true, pt != null ? pt.GetRouteEndSelectionLabel() : "A");
        }

        PressableButton pb = arsisBtn != null ? arsisBtn.GetPressableButton() : buttonObject.GetComponentInChildren<PressableButton>(true);
        if (pb == null)
            return;

        for (int i = 0; i < pb.OnClicked.GetPersistentEventCount(); i++)
            pb.OnClicked.SetPersistentListenerState(i, UnityEngine.Events.UnityEventCallState.Off);
        pb.OnClicked.RemoveAllListeners();
        pb.OnClicked.AddListener(CycleRouteEndFromMenu);
    }

    private void ConfigureOrientNorthButton(GameObject buttonObject)
    {
        if (buttonObject == null)
            return;
        ARSIS.UI.Button arsisBtn = buttonObject.GetComponent<ARSIS.UI.Button>();
        if (arsisBtn != null)
        {
            arsisBtn.SetIcon(false, string.Empty, string.Empty);
            arsisBtn.SetText(true, "Orient North");
        }

        PressableButton pb = arsisBtn != null ? arsisBtn.GetPressableButton() : buttonObject.GetComponentInChildren<PressableButton>(true);
        if (pb != null)
        {
            // Clone carries Find Path persistent listener(s); disable all of them so this button cannot toggle path session.
            for (int i = 0; i < pb.OnClicked.GetPersistentEventCount(); i++)
                pb.OnClicked.SetPersistentListenerState(i, UnityEngine.Events.UnityEventCallState.Off);
            pb.OnClicked.RemoveAllListeners();
            pb.OnClicked.AddListener(OrientNorthFromMenu);
        }
    }

    /// <summary>
    /// Taller hit targets for Find Path / Stop Path and Orient North (layout + optional BoxCollider on the shared button prefab).
    /// </summary>
    private void ApplyPrimaryNavigationButtonSizing()
    {
        if (toggleCapture != null)
            ApplyLayoutSizingToButtonRoot(toggleCapture.transform, PrimaryNavButtonMinHeight, PrimaryNavButtonPreferredHeight);

        Transform pathParent = toggleCapture != null ? toggleCapture.transform.parent : null;
        if (pathParent == null)
            return;
        Transform routeEnd = pathParent.Find("RouteEndButton");
        if (routeEnd != null)
            ApplyLayoutSizingToButtonRoot(routeEnd, PrimaryNavButtonMinHeight, PrimaryNavButtonPreferredHeight);
        Transform orient = pathParent.Find("OrientNorthButton");
        if (orient != null)
            ApplyLayoutSizingToButtonRoot(orient, PrimaryNavButtonMinHeight, PrimaryNavButtonPreferredHeight);
    }

    private static void ApplyLayoutSizingToButtonRoot(Transform buttonRoot, float minHeight, float preferredHeight)
    {
        if (buttonRoot == null)
            return;
        var le = buttonRoot.GetComponent<LayoutElement>();
        if (le == null)
            le = buttonRoot.gameObject.AddComponent<LayoutElement>();
        le.minHeight = minHeight;
        le.preferredHeight = preferredHeight;
        le.flexibleWidth = Mathf.Max(le.flexibleWidth, 1f);
    }

    void Update()
    {
        if (ResolvePathTest() != null)
            SyncFindPathButton();
        else if (TranslationController.S != null)
            SetCaptureButton(TranslationController.S.IsPathCapture());
    }
}
