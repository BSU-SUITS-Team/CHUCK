using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Realtime;
using Photon.Pun;
using System;

public class PhotonRPCLinks : MonoBehaviourPun
{
    public static PhotonRPCLinks singleton = null;
    public static PhotonRPCLinks getSingleton() { return singleton; }
    public Material lineRendererDefaultMaterial = null;
    private Stack<LineRenderer> lineRenderers = null;

    public GameObject ArrowPrefab = null;
    public GameObject CirclePrefab = null;
    public GameObject RectanglePrefab = null;
    public GameObject XPrefab = null;

    public GameObject crewMemberGO = null;
    public Vector3 newCrewMemberPos = Vector3.zero;
    public Quaternion newCrewMemberRot = Quaternion.identity;

    public GameObject headsetCameraGO = null;
    public float lastHeadsetSendUpdate = 0;
    public enum iconType { ARROW, CIRCLE, RECTANGLE, X };

    public GameObject debugCube; 

    // Start is called before the first frame update
    void Start()
    {
        if (!singleton)
            singleton = this;
        else
            Debug.LogError("PhotonRPCLinks DUPLICATE SINGLETONS ATTEMPTED!");
        lineRenderers = new Stack<LineRenderer>();
    }

    private void FixedUpdate()
    {
        //Interpolate new crew member position
        if (!crewMemberGO)
            Debug.LogError("crewMemberGO not configured properly on RPC Link");
        crewMemberGO.transform.position = Vector3.Lerp(crewMemberGO.transform.position, newCrewMemberPos, 0.01f);
        crewMemberGO.transform.rotation = Quaternion.Lerp(crewMemberGO.transform.rotation, newCrewMemberRot, 0.01f);

        if (headsetCameraGO)//if this is filled then we have are the crew member side
        {
            if (lastHeadsetSendUpdate + 1.0f < Time.realtimeSinceStartup)
            {
                lastHeadsetSendUpdate = Time.realtimeSinceStartup;
                sendCrewMemberLocation(headsetCameraGO.transform.position, headsetCameraGO.transform.rotation);
            }
        }
    }

    public void SendLineRenderer(LineRenderer lr)
    {
        Vector3[] verts = new Vector3[lr.positionCount];
        lr.GetPositions(verts);
        PhotonView pv = this.photonView;

        Color32 col = lr.material.color;
        float r = col.r;
        float g = col.g;
        float b = col.b;

        float width = lr.startWidth;
        pv.RPC("receiveLineRenderer", RpcTarget.Others, (object)r, (object)g, (object)b, (object)verts, (object)width);
    }

    [PunRPC]
    void receiveLineRenderer(Single r, Single g, Single b, Vector3[] verts, Single width)
    {
        Debug.Log("Got a line renderer of length " + verts.Length);
        GameObject go = new GameObject();
        go.transform.parent = this.gameObject.transform;
        LineRenderer lr = go.AddComponent<LineRenderer>();
        lr.material = new Material(lineRendererDefaultMaterial);//copy
        lr.material.color = new Color(r, g, b);
        lr.startWidth = width;
        lr.endWidth = width;
        lr.widthMultiplier = 1.0f;
        lr.endColor = lr.startColor = new Color(r, g, b);
        lr.positionCount = verts.Length;
        lr.SetVertexCount(verts.Length);
        for (int i = 0; i < verts.Length; i++)
        {
            lr.SetPosition(i, verts[i]);
        }

        go.SetActive(true);
        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(lr.material.color, 0.0f), new GradientColorKey(lr.material.color, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f), new GradientAlphaKey(1.0f, 1.0f) }
        );
        lr.colorGradient = gradient;
        lineRenderers.Push(lr);
    }

    public void SendLineRendererUndo()
    {
        PhotonView pv = this.photonView;
        pv.RPC("receiveLineRendererUndo", RpcTarget.Others);
    }

    [PunRPC]
    void receiveLineRendererUndo()
    {
        Debug.Log("Line renderer undo called");
        LineRenderer lr = lineRenderers.Pop();
        lr.enabled = false;
        Destroy(lr.gameObject);
    }

    public void sendIconSpawn(Vector3 pos, Quaternion rot, iconType type)
    {
        PhotonView pv = this.photonView;
        pv.RPC("receiveIconSpawn", RpcTarget.All, (object)pos, (object)rot, (object)type);
    }

    [PunRPC]
    public void receiveIconSpawn(Vector3 pos, Quaternion rot, iconType type)
    {
        GameObject selectedPrefab = null;
        switch (type)
        {
            case (iconType.ARROW):
                selectedPrefab = ArrowPrefab;
                break;
            case (iconType.CIRCLE):
                selectedPrefab = CirclePrefab;
                break;
            case (iconType.RECTANGLE):
                selectedPrefab = RectanglePrefab;
                break;
            case (iconType.X):
                selectedPrefab = XPrefab;
                break;
            default:
                selectedPrefab = null;
                Debug.LogError("THE WRONG ENUM WAS USED SOMEHOW! MIND BLOWN MEME GOES HERE");
                break;
        };

        if (selectedPrefab == null)
        {
            Debug.LogError("PREFABS MISCONFIGURED ON RPC LINKS");
            return;
        }
        GameObject.Instantiate(selectedPrefab, pos, rot, this.transform);
    }
    public void sendCrewMemberLocation(Vector3 pos, Quaternion rot)
    {
        debugCube.transform.position = pos;
        debugCube.transform.rotation = rot; 
        PhotonView pv = this.photonView;
        pv.RPC("receiveCrewMemberLocation", RpcTarget.Others, (object)pos, (object)rot);
    }
    [PunRPC]
    public void receiveCrewMemberLocation(Vector3 pos, Quaternion rot)
    {
        if (!crewMemberGO)
            Debug.LogError("crewMemberGO not configured properly on RPC Link");
        newCrewMemberPos = pos;
        newCrewMemberRot = rot;
    }
}