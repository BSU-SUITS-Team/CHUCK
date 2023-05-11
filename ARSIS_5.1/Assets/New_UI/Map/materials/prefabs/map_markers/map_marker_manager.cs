using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class map_marker_manager : MonoBehaviour
{
    public int pointerDist;

    public GameObject player;
    public GameObject mm_follow_objects;
    //public GameObject mm_tags_parent;

    public GameObject tag_anchor;

    public minimap_icon icon;
    public minimap_pointer pointer;
    public minimap_tag tag;

    public GameObject pointer_tip;
    public GameObject beacon;

    void Start () {
        pointer.gameObject.transform.parent = mm_follow_objects.transform;
        //tag.gameObject.transform.parent = mm_tags_parent.transform;
        //tag.gameObject.transform.parent = pointer.gameObject.transform;
        tag.gameObject.transform.parent = tag_anchor.transform;
    }

    void Update () {
        float distance = Vector3.Distance (icon.gameObject.transform.position, player.transform.position);
        if ( distance > pointerDist) {
            pointer.show();
            tag.show();

            tag.updateText ((int) distance + "m");
            pointer.point(this.gameObject);
            tag.updateXY (findTagXY());
        } else {
            pointer.hide();
            tag.hide();
        }
    }

    public void setType (int id) {
        Renderer pr = pointer_tip.GetComponent<Renderer>();
        Renderer br = beacon.GetComponent<Renderer>();

        // change color based on id
        Color waypoint_color = Color.yellow;
        switch (id)
        {
            case 0 :
                waypoint_color = Color.red;
                break;
            case 1 :
                waypoint_color = Color.blue;
                break;
            case 2 :
                waypoint_color = Color.green;
                break;
            default :
                break;
        }


        pr.material.SetColor("_Color", waypoint_color);
        br.material.SetColor("_Color", waypoint_color);
    }

    Vector2 findTagXY () {

        // depreciated



        // returns an xy between -100,-100 and 100,100
        
        // Vector3 pos = icon.gameObject.transform.position - player.transform.position;
        // float x = (pos.x > 0)   ? (pos.x > 50) ? 1 : pos.x/50
        //                         : (pos.x < -50) ? -1 : pos.x/50;

        // float y = (pos.z > 0)   ? (pos.z > 50) ? 1 : pos.z/50
        //                         : (pos.z < -50) ? -1 : pos.z/50;

        // // rotaion\\
        // float angle = player.transform.eulerAngles.z;
        // float c = (float) Math.Cos(angle);
        // float s = (float) Math.Sin(angle);

        // float rotX = x*c - y*s;
        // float rotY = x*s + y*s;



        
        //float angle = pointer.gameObject.transform.eulerAngles.y / 100 ;
        //print (angle + "");
    //    int xSign = (angle < .9 ||  angle > 2.7 ) ? 1 : -1;
    //    int ySign = (angle < 1.8 ) ? 1 : -1;

        //float x = (float) Math.Sin( angle ) ;//* xSign;
        //float y = (float) Math.Cos( angle ) ;//* ySign;

        //print (x + " " + y);


        //return new Vector2 (x, y) * 100;
        //return new Vector2 (rotX, rotY) * 100;
        return new Vector2(0,0);
    }
}
