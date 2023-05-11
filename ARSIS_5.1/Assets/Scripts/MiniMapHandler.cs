using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MiniMapHandler : MonoBehaviour
{
    private static MiniMapHandler singleton = null;
    public GameObject miniMapAnchor = null;
    public List<MapElement> mapElements = null;
    public GameObject meshPrefab = null;
    public Material meshMaterial = null;
    private Vector3 startPosition = Vector3.zero;
    public class MapElement
    {
        public GameObject myInstance = null;
        public float mySpawnTime = 0.0f;
    }

    void Start()
    {
        if (singleton)
            Debug.LogError("Multiple minimaphandler singletons creation attempts were made. Much fail, very bad.");
        singleton = this;
        mapElements = new List<MapElement>();
        //startPosition = this.gameObject.transform.localPosition;//why doesn't this work!?
        startPosition = new Vector3(0.0f, 0.0f, 2.0f);
    }

    public static MiniMapHandler getSingleton()
    {
        if (!singleton)
            Debug.LogError("minimaphandler singleton accessed prior to creation");

        return singleton;
    }

    public void Update()
    {



        //movement with the player

        miniMapAnchor.transform.localPosition = new Vector3(-PhotonRPCLinks.getSingleton().crewMemberGO.transform.position.x,
            0,
            -PhotonRPCLinks.getSingleton().crewMemberGO.transform.position.z);

        //rotation with the player
        miniMapAnchor.transform.localRotation = Quaternion.Euler(new Vector3(0, PhotonRPCLinks.getSingleton().crewMemberGO.transform.rotation.eulerAngles.y + 90.0f, 0)); // the math is only making sense in my head a little bit right now. Dan tired.

    }

    public void addElement(Mesh mesh, Vector3 pos, Quaternion rot)
    {
        MapElement element = new MapElement();
        mapElements.Add(element);
        element.myInstance = GameObject.Instantiate(meshPrefab, pos, rot, this.gameObject.transform);
        element.myInstance.transform.SetParent(miniMapAnchor.transform, false);
        element.mySpawnTime = Time.realtimeSinceStartup;
        element.myInstance.GetComponent<MeshFilter>().mesh = mesh;
        element.myInstance.GetComponent<MeshRenderer>().material = meshMaterial;
        //do mesh limiting each time we add one, since we are doing stuff already. Keeps the hololens from getting bogged down.
        List<MapElement> deleteList = new List<MapElement>();
        for (int i = 0; i < mapElements.Count; i++)
        {
            if (mapElements[i].mySpawnTime + 60.0f < Time.realtimeSinceStartup)
            {
                deleteList.Add(mapElements[i]);
            }
        }
        for (int i = 0; i < deleteList.Count; i++)
        {
            Destroy(deleteList[i].myInstance);
            mapElements.Remove(deleteList[i]);
        }

    }

}
