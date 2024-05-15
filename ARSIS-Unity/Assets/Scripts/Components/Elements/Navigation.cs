using MixedReality.Toolkit;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using System.Linq;
using MixedReality.Toolkit.UX.Experimental;

public class Navigation : MonoBehaviour
{
    [SerializeField] RectTransform image;
    [SerializeField] GameObject map;
    [SerializeField] GameObject imageObj;
    [SerializeField] RectTransform mapTrans;
    [SerializeField] GameObject pinPrefab;
    private float maxScale = 2f;
    private float minScale = 0.1f;
    private float pinSize = 5f;
    private float pinScale = 0.2f;

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
        pin.transform.SetParent(map.transform, false);
        RectTransform pinTrans = pin.GetComponent<RectTransform>();
        pinTrans.localScale = Vector3.one * pinScale;
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, pinSize);
        pinTrans.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Top, 0, pinSize);
        pinTrans.anchoredPosition = anchored;
        pin.transform.SetParent(image.transform, true);
    }

    private Vector2 CalculateAnchor(Vector3 hit)
    {
        Vector3[] corners = new Vector3[4];
        image.GetWorldCorners(corners);
        Vector3 bottomLeft = corners[0];
        Vector3 topLeft = corners[1];
        Vector3 topRight = corners[2];
        Vector3 direction = hit - topLeft;
        Vector3 width = topRight - topLeft;
        Vector3 height = bottomLeft - topLeft;
        float widthFactor = Vector3.Dot(direction, width) / width.magnitude / width.magnitude;
        float heightFactor = Vector3.Dot(direction, height) / height.magnitude / height.magnitude;
        return new Vector2(widthFactor * mapTrans.sizeDelta.x, heightFactor * -mapTrans.sizeDelta.y);
    }

    public void HandleSelect(SelectExitEventArgs e)
    {
        IXRSelectInteractor interactor = e.interactorObject;
        IXRSelectInteractable interactable = e.interactableObject;
        Transform trans = interactor.GetAttachTransform(interactable).parent;
        RaycastHit hit;
        if (!Physics.Raycast(trans.position, trans.forward, out hit)) return;
        if (hit.transform.gameObject != map.transform.gameObject) return;
        Vector2 anchored = CalculateAnchor(hit.point);
        CreatePin(anchored);
    }
}
