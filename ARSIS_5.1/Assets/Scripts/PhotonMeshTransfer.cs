using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Realtime;
using Photon.Pun;
public class PhotonMeshTransfer : MonoBehaviourPun
{
    public static PhotonMeshTransfer singleton = null;
    public float testLastSendGametime = 0;
    public GameObject meshPrefab = null;
    public Material meshMaterial = null;


    void Start()
    {
        if (!singleton)
            singleton = this;
        else
            Debug.LogError("Photon Mesh Transfer DUPLICATE SINGLETONS ATTEMPTED!");
        testLastSendGametime = Time.realtimeSinceStartup;
        if (!meshPrefab)
        {
            Debug.LogError("You screwed up when setting up this gameobject. meshPrefab is null.");
        }
    }

    void FixedUpdate()
    {
        float timeDif = Time.realtimeSinceStartup - testLastSendGametime;
        if (timeDif > 0.1f)
        {
            testLastSendGametime = Time.realtimeSinceStartup;
            //uncomment this block for generating arbitrary mesh test data

            //generate arbitrary mesh data, in this case a triangle
            //2020-05-31 - this still fracking works - Dan
            /*
            Vector3 pos = UnityEngine.Random.insideUnitSphere * UnityEngine.Random.Range(0, 100.0f);
            Vector3[] verts = { UnityEngine.Random.insideUnitSphere * UnityEngine.Random.Range(0, 100.0f), UnityEngine.Random.insideUnitSphere * UnityEngine.Random.Range(0, 100.0f), UnityEngine.Random.insideUnitSphere * UnityEngine.Random.Range(0, 100.0f) };
            verts[0].y *= 0.001f;
            verts[1].y *= 0.001f;
            verts[2].y *= 0.001f;
            int[] indexors = { 0, 1, 2, 1, 0, 2 };
            PhotonView pv = this.photonView;
            Quaternion rot = this.gameObject.transform.rotation;

            //minimap testing
            Mesh meesh = new Mesh();
            meesh.vertices = verts;
            meesh.SetIndices(indexors, MeshTopology.Triangles, 0);
            meesh.RecalculateBounds();
            meesh.RecalculateNormals();
            meesh.RecalculateTangents();
            MiniMapHandler.getSingleton().addElement(meesh, verts[0], rot);
            */
            //network testing
            //pv.RPC("receiveMeshData", RpcTarget.All, pos, (object)rot, (object)verts, (object)indexors);

        }
    }

    public void sendMesh(Vector3 pos, Quaternion rot, Mesh mesh)
    {
        List<Vector3> verts = new List<Vector3>();
        mesh.GetVertices(verts);
        Vector3[] vertout = verts.ToArray();
        int[] indicies = mesh.GetIndices(0);
        PhotonView pv = this.photonView;
        pv.RPC("receiveMeshData", RpcTarget.Others, pos, (object)rot, (object)vertout, (object)indicies);
    }


    [PunRPC]
    void receiveMeshData(Vector3 pos, Quaternion rot, Vector3[] verts, int[] indecieies)
    {
        Debug.Log(pos);
        Debug.Log(verts[2]);
        Debug.Log(indecieies[3]);

        GameObject newMesh = GameObject.Instantiate(meshPrefab);
        newMesh.transform.position = pos;
        newMesh.transform.rotation = rot;
        Mesh meesh = new Mesh();
        meesh.vertices = verts;
        meesh.SetIndices(indecieies, MeshTopology.Triangles, 0);
        meesh.RecalculateBounds();
        meesh.RecalculateNormals();
        meesh.RecalculateTangents();
        newMesh.GetComponent<MeshFilter>().mesh = meesh;
        newMesh.GetComponent<MeshRenderer>().material = meshMaterial;
        //MiniMapHandler.getSingleton().addElement(meesh, pos, rot);
    }

    public static PhotonMeshTransfer getSingleton()
    {
        return singleton;
    }
}

