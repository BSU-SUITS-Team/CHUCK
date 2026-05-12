using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using System.Linq;
using ARSIS.EventManager;
using MixedReality.Toolkit.UX;

public class Navigation : MonoBehaviour, IRenderable
{
    /// <summary>Most recently enabled Navigation panel — used to refresh GridManager when starting a path from an arbitrary pose.</summary>
    private static Navigation s_lastActiveInstance;

    [SerializeField] RectTransform image;
    [SerializeField] RectTransform map;
    [SerializeField] GameObject pinPrefab;
    [SerializeField] ARSIS.UI.Button selectButton;
    [SerializeField] ARSIS.UI.Button toggleCapture;
    [SerializeField, Tooltip("Optional. If unset, uses PathTest.Instance for Find Path / Stop Path.")]
    private PathTest pathTest;
    [SerializeField, Tooltip("Optional north marker on this prefab (e.g. empty under Map). Copied to PathTest.mapNorthReference whenever this panel syncs. Leave empty to use PathTest’s own assignment or default map-up north.")]
    private Transform mapNorthReference;
    [SerializeField, Tooltip("Optional distance line. Leave empty to auto-create PathDistanceReadout under the Controls column when ensurePathDistanceInSidebar is on.")]
    private TextMeshProUGUI pathDistanceReadout;
    [SerializeField, Tooltip("Adds a row under Find Path / pin buttons on the Navigation panel and binds it to PathTest.")]
    private bool ensurePathDistanceInSidebar = true;

    private List<BaseArsisEvent> pins = new();
    private bool changed = true;
    private float maxScale = 2f;
    private float minScale = 0.05f;
    private float pinWidth = 60f;
    private float pinHeight = 120f;
    private float pinScale = 2f;
    private float selectProximity = 240f;
    private Pins selectedPin;
    private bool isPinActive = false;
    private bool isCapture = false;
    private bool findPathButtonSynced;

    private PathTest ResolvePathTest() => pathTest != null ? pathTest : PathTest.Instance;

    /// <summary>Call when PathTest starts a session so the grid uses this menu’s map plane (same as at Start / after floating placement).</summary>
    public static void RefreshPathfindingMapBindingStatic()
    {
        if (s_lastActiveInstance != null)
            s_lastActiveInstance.SyncPathfindingToActiveMapPlane();
    }

    private void OnEnable()
    {
        s_lastActiveInstance = this;
        SyncPathfindingToActiveMapPlane();
    }

    private void OnDisable()
    {
        if (s_lastActiveInstance == this)
            s_lastActiveInstance = null;
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

    public void ToggleActivePin()
    {
        isPinActive = !isPinActive;
        selectButton.SetIcon(isPinActive ? "Icon 16" : "Icon 14");
        selectButton.SetText((isPinActive ? "Hide " : "Show ") + selectedPin.data.properties.name);
    }

    private void SetSelectedPin(Pins pin)
    {
        selectedPin = pin;
        if (selectedPin == null)
        {
            selectButton.SetIcon("Icon 80");
            selectButton.SetText("No Pin Selected");
            return;
        }
        isPinActive = false;
        selectButton.SetIcon("Icon 14");
        selectButton.SetText("Show " + selectedPin.data.properties.name);
    }

    public void adjustScale(float adjust)
    {
        if (image == null) return;
        float scale = image.localScale.x;
        float newScale = Mathf.Clamp(scale + adjust, minScale, maxScale);
        image.localScale = new Vector3(newScale, newScale, 0);
    }

    private GameObject CreatePin(Vector2 anchored)
    {
        GameObject pin = Instantiate(pinPrefab);
        pin.transform.SetParent(image, false);
        pin.transform.localScale = Vector3.zero * pinScale;
        pin.transform.SetParent(map, true);
        RectTransform pinTrans = pin.GetComponent<RectTransform>();
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, pinWidth);
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Top, 0, pinHeight);
        pinTrans.anchoredPosition = anchored;
        pin.transform.SetParent(image, true);
        return pin;
    }

    private void PlacePoint(Pins point)
    {
        GameObject pin = Instantiate(pinPrefab);
        pin.transform.SetParent(image, false);
        RectTransform pinTrans = pin.GetComponent<RectTransform>();
        pinTrans.localScale = Vector3.one * pinScale;
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, pinWidth);
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Top, 0, pinHeight);
        pinTrans.localPosition = Vector3.zero;
        pinTrans.anchoredPosition = new Vector2(point.data.properties.x, -point.data.properties.y + pinHeight);
        BeaconObject beacon = pin.GetComponent<BeaconObject>();
        beacon.SetText(point.data.properties.name);
        beacon.SetDistance("XXX meters");
    }

    private Vector2 CalculateAnchor(Vector3 hit)
    {
        Vector3[] corners = new Vector3[4];
        map.GetWorldCorners(corners);
        Vector3 bottomLeft = corners[0];
        Vector3 topLeft = corners[1];
        Vector3 topRight = corners[2];
        Vector3 direction = hit - topLeft;
        Vector3 width = topRight - topLeft;
        Vector3 height = bottomLeft - topLeft;
        float widthFactor = Vector3.Dot(direction, width) / width.magnitude / width.magnitude;
        float heightFactor = Vector3.Dot(direction, height) / height.magnitude / height.magnitude;
        Debug.Log($"x: {widthFactor * map.sizeDelta.x}, y: {heightFactor * -map.sizeDelta.y}");
        return new Vector2(widthFactor * map.sizeDelta.x, heightFactor * -map.sizeDelta.y);
    }

    public void SelectPin(Vector2 point, float proximity)
    {
        GameObject pin = CreatePin(point);
        RectTransform cursor = pin.GetComponent<RectTransform>();
        Pins closest = null;
        float closestProximity = Mathf.Infinity;
        IEnumerable<Pins> points = pins.Where(e => e is Pins location && location.data.type.Equals("Point")).OfType<Pins>();
        foreach (Pins p in points)
        {
            Vector2 a = new Vector2(p.data.properties.x, -p.data.properties.y);
            float distance = Vector2.Distance(a, cursor.anchoredPosition);
            if (distance <= proximity)
            {
                if (closest == null || distance <= closestProximity)
                {
                    closest = p;
                    closestProximity = distance;
                }
            }
        }
        SetSelectedPin(closest);
        Destroy(pin);
        Debug.Log($"selectedPin: {selectedPin}");
    }

    public void HandleSelect(SelectExitEventArgs e)
    {
        IXRSelectInteractor interactor = e.interactorObject;
        IXRSelectInteractable interactable = e.interactableObject;
        Transform trans = interactor.GetAttachTransform(interactable).parent;
        RaycastHit hit;
        if (!Physics.Raycast(trans.position, trans.forward, out hit)) return;
        if (hit.transform.gameObject != map.gameObject) return;
        Debug.Log($"hit: {hit.point}");
        Vector2 anchored = CalculateAnchor(hit.point);
        Debug.Log($"anchored: {anchored}");
        //CreatePin(anchored);
        SelectPin(anchored, selectProximity);
    }

    void RemovePins()
    {
        foreach (Transform child in image.transform)
            Destroy(child.gameObject);
    }

    void Awake()
    {
        if (pathDistanceReadout == null && ensurePathDistanceInSidebar)
            TryInjectPathDistanceSidebar();
        TryEnsureOrientNorthButton();
    }

    void Start()
    {
        SyncPathfindingToActiveMapPlane();
        PathTest pt = ResolvePathTest();
        if (pathDistanceReadout != null && pt != null)
            pt.BindDistanceReadout(pathDistanceReadout);
        if (pt != null)
        {
            bool session = pt.IsPathSessionActive();
            findPathButtonSynced = session;
            string icon = session ? "Icon 135" : "Icon 128";
            string label = session ? "Stop Path" : "Find Path";
            toggleCapture.SetIcon(true, icon, label);
        }
        else if (TranslationController.S != null)
            SetCaptureButton(TranslationController.S.IsPathCapture());
        EventDatastore.Instance.AddHandler("pins", this);
    }

    void OnDestroy()
    {
        if (pathDistanceReadout != null && PathTest.Instance != null)
            PathTest.Instance.UnbindDistanceReadout(pathDistanceReadout);
        EventDatastore.Instance.RemoveHandler("pins", this);
    }

    /// <summary>Called after <see cref="FloatingMenuFromPrefab"/> positions the panel so PathTest uses this instance's map plane (not scene origin).</summary>
    public void NotifyMenuPlacedInFrontOfUser()
    {
        SyncPathfindingToActiveMapPlane();
    }

    private void SyncPathfindingToActiveMapPlane()
    {
        // Chart + texture live on `map`; `image` is often a zoom/pan parent. Binding `image` skews the grid vs the painted quad.
        RectTransform plane = map != null ? map : image;
        if (plane == null)
            return;
        PathTest pt = ResolvePathTest();
        if (pt == null)
            return;
        pt.BindActiveNavigationMap(plane);
        if (mapNorthReference != null)
        {
            pt.mapNorthReference = mapNorthReference;
            pt.RefreshMapNorthReferenceVisibility();
        }
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
                TryEnsureOrientNorthButtonUnderControls(rt);
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
        rowLe.minHeight = 48f;
        rowLe.preferredHeight = 72f;

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
        TryEnsureOrientNorthButtonUnderControls(controls);
    }

    private void TryEnsureOrientNorthButton()
    {
        foreach (RectTransform rt in GetComponentsInChildren<RectTransform>(true))
        {
            if (rt.name != "Controls")
                continue;
            if (rt.GetComponent<VerticalLayoutGroup>() == null)
                continue;
            TryEnsureOrientNorthButtonUnderControls(rt);
            return;
        }
    }

    private void TryEnsureOrientNorthButtonUnderControls(RectTransform controls)
    {
        if (controls == null || toggleCapture == null)
            return;

        Transform legacyRow = controls.Find("OrientNorthRow");
        if (legacyRow != null)
            Destroy(legacyRow.gameObject);

        Transform pathParent = toggleCapture.transform.parent;
        if (pathParent == null)
            return;

        Transform existing = pathParent.Find("OrientNorthButton");
        if (existing != null)
        {
            ConfigureOrientNorthButton(existing.gameObject);
            return;
        }

        GameObject clone = Instantiate(toggleCapture.gameObject, pathParent);
        clone.name = "OrientNorthButton";
        ConfigureOrientNorthButton(clone);
        clone.transform.SetSiblingIndex(toggleCapture.transform.GetSiblingIndex() + 1);
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

    void Update()
    {
        if (ResolvePathTest() != null)
            SyncFindPathButton();
        else if (TranslationController.S != null)
            SetCaptureButton(TranslationController.S.IsPathCapture());
        if (!changed || pins.Count == 0) return;
        changed = false;
        IEnumerable<Pins> points = pins.Where(e => e is Pins location && location.data.type.Equals("Point")).OfType<Pins>();
        RemovePins();
        foreach (Pins point in points)
        {
            PlacePoint(point);
        }
    }

    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        changed = true;
        pins = data;
    }
}
