using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class minimap_manager : MonoBehaviour
{
    public static minimap_manager S;
    public int currentMap = 0; 

    private const int MINIMAP_BEACONS_LAYER = 29;
    private const int NUM_CHECKPOINTS = 4;
    private const int NUM_MAPS = 2;

    [Header("Maps")]
    public Texture[] maps;
    public Vector2[] map_size_meters;
    public Vector2[] zz_offset_meters;
    public Vector3[,] mission_markers = {   { new Vector3(0,0,1), new Vector3(0,14.08f,0), new Vector3(-9.66f, -2.68f, 2), new Vector3(0,0,0) },    //backup map
                                            { new Vector3(0,0,0), new Vector3(0,4.75f,1), new Vector3(-23.59f, 149.855f, 2), new Vector3(0,0,0) }     //primary map
                                        };  // first two values in vect3 are xy coords, last is task type ID 0=lander, 1=sampling, 2=retrieval

    [Header("References")]
    public GameObject map_render_parent;
    public GameObject lander_marker;
    public GameObject player;
    public GameObject mm_follow_objects;
    public Camera mm_camera;
    //public GameObject mm_tags_parent;

    private Vector2 zeroZeroOffset;
    private RawImage map_render;
    private ArrayList markers;
    

    void Start()
    {
        S = this; 

        map_render = map_render_parent.GetComponent<RawImage>();
        markers = new ArrayList();

        LoadMap(currentMap);
    }

    //  custom funcs
    /*
     *  sets up the data for a minimap
     *
     *  t = a texture representing the map, size is unimportant
     *  finalSize = intended size of the map, in meters
     *  zeroZeroOffset = how far the 0,0 position is from the center of the
     */
    public void LoadMap (int index) {
        RectTransform rt = map_render_parent.GetComponent<RectTransform>();
        rt.position += new Vector3 (zz_offset_meters[index].x, 0, zz_offset_meters[index].y);

        map_render.texture = maps[index];
        rt.sizeDelta = map_size_meters[index];

        loadAllMarkers(index);
    }

    public void placeMarker (Vector3 vectorizedInput) {
        placeMarker((int)vectorizedInput.z, new Vector2 (vectorizedInput.x, vectorizedInput.y));
    }
    public void placeMarker (int markerId, Vector2 position) {
        GameObject new_marker = GameObject.Instantiate(lander_marker);
        map_marker_manager mmm = new_marker.GetComponent<map_marker_manager>();
        mmm.player = player;
        mmm.mm_follow_objects = mm_follow_objects;
        
        //  set color based on id
        new_marker.GetComponent<map_marker_manager>().setType(markerId);

        new_marker.transform.position = new Vector3 (position.x, 0, position.y);
        markers.Add(new_marker);
    }
    
    public void loadAllMarkers (int index) {
        for (int i=0; i<NUM_CHECKPOINTS; i++) {
            placeMarker(mission_markers[index, i]);
        }
    }

}
