using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class MapManager : MonoBehaviour
{
    private class Icon {
        public BeaconsManager.BeaconType bt;
        public BeaconsManager.Beacon b;
        public Transform t;
        public SpriteRenderer sr;
    }

    [Serializable]
    public class MapObject
    {
        public GameObject game_object;

        [HideInInspector]
        public Vector2 lat_long_at_center;
        [HideInInspector]
        public Map m;
        [HideInInspector]
        public RawImage ri;
        [HideInInspector]
        public Transform t;

        public void init ()
        {
            ri = game_object.GetComponent<RawImage>();
            t = game_object.transform;
            m = game_object.GetComponent<Map>();
            lat_long_at_center = m.lat_long_at_center;
        }

        public void update ()
        {
            t.localScale = Vector3.one * m.scale;
        }
    }

    public GameObject icon_prefab;
    public GameObject icons_layer;
    public MapObject map;
    public float icon_size;

    void Start ()
    {
        map.init();
        //SLAPUtils.SetMap(map.m);
    }
    void Update ()
    {
        map.update();
    }

    public void AddIcon (BeaconsManager.BeaconType bt, BeaconsManager.Beacon b)
    {
        Icon i = new Icon();
        GameObject tmp = Instantiate(icon_prefab);
        i.bt = bt;
        i.b = b;
        i.t = tmp.GetComponent<Transform>();
        i.sr = tmp.GetComponent<SpriteRenderer>();
        i.t.localScale = Vector3.one * map.m.scale * icon_size;

        if (i.t == null || i.sr == null) return;

        i.t.SetParent(icons_layer.transform);

        i.t.position = map.t.position;
        //Vector2 offset = SLAPUtils.MapCoordsOffsetFromLatLong(i.b.lat_long, map.m);
        Vector3 new_pos = i.t.position;
        //new_pos.x += offset.x;
        //new_pos.y += offset.y;
        //new_pos.z -= .01f;
        //i.t.position = new_pos;
    }
}
