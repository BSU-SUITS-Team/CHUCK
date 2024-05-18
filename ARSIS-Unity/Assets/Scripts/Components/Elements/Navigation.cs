using MixedReality.Toolkit;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using System.Linq;
using MixedReality.Toolkit.UX.Experimental;
using ARSIS.EventManager;
using ARSIS.UI;

public class Navigation : MonoBehaviour, IRenderable
{
    [SerializeField] RectTransform image;
    [SerializeField] RectTransform map;
    [SerializeField] GameObject pinPrefab;
    [SerializeField] Button selectButton;
    [SerializeField] Button toggleCapture;

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

    private void SetCaptureButton(bool isPathCapture)
    {
        if (isCapture == isPathCapture) return;
        string icon = isPathCapture ? "Icon 135" : "Icon 128";
        string label = isPathCapture ? "Stop Path" : "Record Path";
        toggleCapture.SetIcon(true, icon, label);
        isCapture = isPathCapture;
    }

    public void TogglePathCapture()
    {
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
        Locations2.Instance.SetActivePin(selectedPin);
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

    void Start()
    {
        SetCaptureButton(TranslationController.S.IsPathCapture());
        EventDatastore.Instance.AddHandler("pins", this);
    }

    void OnDestroy()
    {
        EventDatastore.Instance.RemoveHandler("pins", this);
    }

    void Update()
    {
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
