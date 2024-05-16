using MixedReality.Toolkit;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using System.Linq;
using MixedReality.Toolkit.UX.Experimental;
using ARSIS.EventManager;

public class Navigation : MonoBehaviour, IRenderable
{
    [SerializeField] RectTransform image;
    [SerializeField] RectTransform map;
    [SerializeField] GameObject pinPrefab;
    private List<BaseArsisEvent> pins = new();
    private bool changed = true;
    private float maxScale = 2f;
    private float minScale = 0.05f;
    private float pinWidth = 60f;
    private float pinHeight = 120f;
    private float pinScale = 2f;

    public void adjustScale(float adjust)
    {
        if (image == null) return;
        float scale = image.localScale.x;
        float newScale = Mathf.Clamp(scale + adjust, minScale, maxScale);
        image.localScale = new Vector3(newScale, newScale, 0);
    }

    private void CreatePin(Vector2 anchored)
    {
        GameObject pin = Instantiate(pinPrefab);
        pin.transform.SetParent(image, false);
        pin.transform.localScale = Vector3.one * pinScale;
        pin.transform.SetParent(map, true);
        RectTransform pinTrans = pin.GetComponent<RectTransform>();
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, pinWidth);
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Top, 0, pinHeight);
        pinTrans.anchoredPosition = anchored;
        pin.transform.SetParent(image, true);
    }

    private void PlacePin(Vector2 anchored)
    {
        GameObject pin = Instantiate(pinPrefab);
        pin.transform.SetParent(image, false);
        RectTransform pinTrans = pin.GetComponent<RectTransform>();
        pinTrans.localScale = Vector3.one * pinScale;
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, pinWidth);
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Top, 0, pinHeight);
        pinTrans.localPosition = Vector3.zero;
        pinTrans.anchoredPosition = new Vector2(anchored.x, anchored.y + pinHeight);
        Debug.Log($"anchored: {anchored}");
        Debug.Log($"localPosition: {pinTrans.localPosition}");
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
        CreatePin(anchored);
    }

    void RemovePins()
    {
        foreach (Transform child in image.transform)
            Destroy(child.gameObject);
    }

    void Start()
    {
        EventDatastore.Instance.AddHandler("pins", this);
    }

    void OnDestroy()
    {
        EventDatastore.Instance.RemoveHandler("pins", this);
    }

    void Update()
    {
        if (!changed || pins.Count == 0) return;
        IEnumerable<Pins> points = pins.Where(e => e is Pins location && location.data.type.Equals("Point")).OfType<Pins>();
        RemovePins();
        foreach (Pins point in points)
        {
            PlacePin(new Vector2(point.data.properties.x, -point.data.properties.y));
        }
        changed = false;
    }

    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        changed = true;
        pins = data;
    }
}
